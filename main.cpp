/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | This software was written by the Machine Perception and Intelligent    |
   |   Robotics Lab, University of Malaga (Spain).                          |
   | Contact: Jose-Luis Blanco  <jlblanco@ual.es>                           |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors                       |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#define MRPT_NO_WARN_BIG_HDR  // allow "large" includes without warning about it
#include <mrpt/gui.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps.h>
#include <mrpt/obs.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/COctreePointRenderer.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/os.h>
#include <mrpt/topography.h>

#include <queue>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::io;
using namespace std;

void gps_coords2map_px(
    const topography::TGeodeticCoords& gps, TPixelCoordf& px);

int main(int argc, char** argv)
{
    // CDisplayWindow3D  win("Data set preview",608,544);  // size multiple of
    // 16 to easy video codecs.
    const int                 IM_W = 640;  // 608;
    const int                 IM_H = 480;  // 544;
    mrpt::opengl::CFBORender  fbo(IM_W, IM_H);
    mrpt::gui::CDisplayWindow win2("Off-screen rendering...");

    // mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1e7;
    mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL(0.1);

    try
    {
        if (argc != 2 && argc != 3)
        {
            cerr << "Usage:\n"
                 << argv[0] << " <RAWLOG FILE> [optional_start_timestamp]\n\n";
            return 1;
        }

        const bool GRAB_VIDEO = true;

        const string rawlog_file = string(argv[1]);
        ASSERT_(mrpt::system::fileExists(rawlog_file));

        const TTimeStamp start_timestamp =
            (argc == 3) ? mrpt::system::time_tToTimestamp(atof(argv[2]))
                        : mrpt::system::time_tToTimestamp(0.0);

        if (argc == 3)
            cout << "Using starting timestamp = "
                 << mrpt::system::dateTimeLocalToString(start_timestamp)
                 << endl;

        // External images:
        CImage::setImagesPathBase(CRawlog::detectImagesDirectory(rawlog_file));

        CFileGZInputStream fil(rawlog_file);

        size_t                 nEntry = 0;
        CActionCollection::Ptr acts;
        CSensoryFrame::Ptr     SF;
        CObservation::Ptr      obs;

        using TMapTime2Lasers =
            std::map<system::TTimeStamp, CObservation2DRangeScan::Ptr>;

        CPose3DInterpolator                    vehPath;
        TMapTime2Lasers                        lstLaserScans;
        std::map<std::string, TMapTime2Lasers> lstLaserScansByName;
        topography::TGeodeticCoords            coords_ref, last_coords;

        // Filter to remove laser points of the car itself:
        CObservation2DRangeScan::TListExclusionAreasWithRanges filterAreas;
        {
            filterAreas.resize(1);
            mrpt::math::CPolygon polygon;
            polygon.AddVertex(-1.9, -1.9);
            polygon.AddVertex(-1.9, 1.9);
            polygon.AddVertex(1.9, 1.9);
            polygon.AddVertex(1.9, -1.9);
            filterAreas[0].first  = polygon;
            filterAreas[0].second = std::make_pair<double, double>(
                -1.5, 1.0);  // [min,max] Z coordinate for removing the points.
        }

        cout << "Parsing rawlog...\n";

        TTimeStamp first_tim        = INVALID_TIMESTAMP;
        bool       waiting_to_first = false;
        TPose3D*   P_prev           = NULL;

        // Build 3D point cloud:
        mrpt::maps::CColouredPointsMap ptsMap;
        ptsMap.insertionOptions.isPlanarMap               = false;
        ptsMap.insertionOptions.fuseWithExisting          = false;
        ptsMap.insertionOptions.minDistBetweenLaserPoints = 0.05;

        // ptsMap.colorScheme.scheme =
        // CColouredPointsMap::cmFromHeightRelativeToSensorJet;
        ptsMap.colorScheme.scheme =
            mrpt::maps::CColouredPointsMap::cmFromHeightRelativeToSensorGray;
        ptsMap.colorScheme.z_min = -2;
        ptsMap.colorScheme.z_max = 10;

        // Keep a queue of images pending to be displayed: this will work as
        //  a delay so we gather enough future GPSs to interpolate the actual
        //  vehicle pose:
        std::queue<CObservationStereoImages::Ptr> latest_imgs;

        const double TIME_WINDOW_SECS = 5;  // secs
        const double N_MIN_IMGS_IN_QUEUE =
            20.0 * TIME_WINDOW_SECS;  // 20fps * secs

        const int GRID_XY_FREQ = 5;

        // Visualize 3D path:

        opengl::CGridPlaneXY::Ptr gl_grid =
            opengl::CGridPlaneXY::Create(-100, 100, -100, 100, 0, GRID_XY_FREQ);
        opengl::CSetOfObjects::Ptr gl_pointcloud =
            opengl::CSetOfObjects::Create();

        opengl::CSetOfObjects::Ptr gl_veh =
            opengl::stock_objects::CornerXYZSimple();

        // This is the only point where we select which lasers to render in 3D
        // as scans:
        map<string, opengl::CPlanarLaserScan::Ptr> gl_laser_scans;
        gl_laser_scans["HOKUYO2"] = opengl::CPlanarLaserScan::Create();
        gl_laser_scans["HOKUYO3"] = opengl::CPlanarLaserScan::Create();

        // gl_laser_scans["HOKUYO1"] = opengl::CPlanarLaserScan::Create();

        //		gl_laser_scans["LASER1"] = opengl::CPlanarLaserScan::Create();
        //		gl_laser_scans["LASER2"] = opengl::CPlanarLaserScan::Create();

        // gl_laser_scans["HOKUYO1"]->setSurfaceColor(0.4,0,1,0.1);
        gl_laser_scans["HOKUYO2"]->setSurfaceColor(0, 0, 1, 0.3);
        gl_laser_scans["HOKUYO3"]->setSurfaceColor(0, 0, 1, 0.3);

        // Add to gl_veh:
        for (map<string, opengl::CPlanarLaserScan::Ptr>::iterator it =
                 gl_laser_scans.begin();
             it != gl_laser_scans.end(); ++it)
        { gl_veh->insert(it->second); }

        opengl::COpenGLScene scene;

        opengl::COpenGLViewport::Ptr view_im1, view_main, view_map;

        CCamera& camera = fbo.getCamera(scene);

        {
            camera.setZoomDistance(50);

            scene.insert(gl_grid);
            scene.insert(gl_pointcloud);
            scene.insert(gl_veh);

            view_im1  = scene.createViewport("im1");
            view_map  = scene.createViewport("map");
            view_main = scene.getViewport("main");

            view_im1->setViewportPosition(
                0, 0.6, 0.4,
                0.4);  // (from_x,from_y) -> (width_x,height_y), in ratios
            view_map->setViewportPosition(0.4, 0.6, 0.6, 0.4);
            view_main->setViewportPosition(0, 0, 1, 0.7);

            view_main->setCustomBackgroundColor(TColorf(0.6, 0.6, 0.6));
        }

        // Grab video:
        unsigned int img_idx       = 0;
        const char*  img_filenames = "out_video/img_%06u.png";
        if (GRAB_VIDEO)
        {
            if (!::system("rm -fr out_video/")) {}
            if (!::system("mkdir out_video")) {}
            cout << "*** GRABBING VIDEO ***\n";
        }

        mrpt::serialization::CArchive::UniquePtr m_rawlog_arch =
            mrpt::serialization::archiveUniquePtrFrom(fil);
        while (CRawlog::getActionObservationPairOrObservation(
                   *m_rawlog_arch, acts, SF, obs, nEntry) &&
               win2.isOpen())
        {
            // Process keys for camera:
            if (mrpt::system::os::kbhit())
            {
                const int c = mrpt::system::os::getch();
                switch (c)
                {
                    case 'q':
                    case 'Q':
                        camera.setZoomDistance(1.2 * camera.getZoomDistance());
                        break;
                    case 'a':
                    case 'A':
                        camera.setZoomDistance(camera.getZoomDistance() / 1.2);
                        break;
                    case 's':
                    case 'S':
                        camera.setAzimuthDegrees(
                            camera.getAzimuthDegrees() - 5.0);
                        break;
                    case 'd':
                    case 'D':
                        camera.setAzimuthDegrees(
                            camera.getAzimuthDegrees() + 5.0);
                        break;
                }
            }

            bool want_to_refresh_view = false;

            if (!obs || obs->timestamp < start_timestamp)
            {
                waiting_to_first = true;
                continue;
            }

            if (waiting_to_first)
            {
                waiting_to_first = false;
                cout << "Reached desired starting point...\n";
            }

            // Process GPS entries:
            if (IS_CLASS(*obs, CObservationGPS) &&
                obs->sensorLabel == "GPS_DELUO")
            {
                CObservationGPS::Ptr o =
                    std::dynamic_pointer_cast<CObservationGPS>(obs);
                if (o->has_GGA_datum() &&
                    o->getMsgByClass<gnss::Message_NMEA_GGA>()
                            .fields.fix_quality >= 1)
                {
                    topography::TGeodeticCoords coord =
                        o->getMsgByClass<gnss::Message_NMEA_GGA>()
                            .getAsStruct<topography::TGeodeticCoords>();
                    if (coords_ref.isClear()) coords_ref = coord;

                    last_coords = coord;

                    TPoint3D P;
                    topography::geodeticToENU_WGS84(coord, P, coords_ref);

                    // Insert? Only if there is a minimum distance to last one,
                    // so we can approximate well the heading:
                    static const double MIN_2D_DIST_TO_ACCEPT = 0.15;

                    bool runInterpolate(false);
                    if (P_prev == NULL) { runInterpolate = true; }
                    else
                    {
                        CPose3D P_prev_pose(*P_prev);
                        if (P_prev_pose.distance2DTo(P.x, P.y) >
                            MIN_2D_DIST_TO_ACCEPT)
                        {
                            runInterpolate = true;
                            cout << "distance to last GPS point: "
                                 << P_prev_pose.distance2DTo(P.x, P.y) << endl;
                        }
                    }
                    if (runInterpolate)
                    {
                        vehPath.insert(
                            o->timestamp, CPose3D(P.x, P.y, P.z, 0, 0, 0));

                        // Now, interpolate between poses and make up the
                        // orientation: Take a reference to the just-inserted
                        // pose:
                        TPose3D& P2 = vehPath.rbegin()->second;
                        if (P_prev)
                        {
                            // Interpolate orientation of P1:
                            const double yaw =
                                atan2(P2.y - P_prev->y, P2.x - P_prev->x);
                            P_prev->yaw = yaw;
                        }
                        P_prev = &P2;  // For next iter:
                    }
                }

                if (first_tim == INVALID_TIMESTAMP)
                    first_tim = o->timestamp;
                else
                {
                    static int decim = 0;
                    if (++decim > 25)
                    {
                        decim = 0;
                        cout << "Processed "
                             << formatTimeInterval(
                                    timeDifference(first_tim, o->timestamp))
                             << " - used "
                             << mrpt::system::getMemoryUsage() /
                                    (1024.0 * 1024.0)
                             << " Mb.\n"
                             << " lat: " << last_coords.lat.getAsString()
                             << " lon: " << last_coords.lon.getAsString()
                             << endl;
                    }
                }
            }
            else if (IS_CLASS(*obs, CObservation2DRangeScan))
            {
                CObservation2DRangeScan::Ptr o =
                    std::dynamic_pointer_cast<CObservation2DRangeScan>(obs);
                if (o->sensorLabel == "HOKUYO2" || o->sensorLabel == "HOKUYO3")
                {
                    o->filterByExclusionAreas(filterAreas);
                    lstLaserScans[o->timestamp] = o;  // Use for point clouds
                }

                if (o->sensorLabel == "HOKUYO1")
                {
                    // static const CPose3D hok1_pos(0.536,0,0.273,0,
                    // DEG2RAD(21.402),DEG2RAD(0.015) );
                    static const CPose3D hok1_pos(
                        0.536, 0, 0.31, 0, DEG2RAD(14), DEG2RAD(0.015));
                    o->setSensorPose(hok1_pos);
                }

                // Use to render cool 3D scans:
                lstLaserScansByName[obs->sensorLabel][o->timestamp] = o;
            }
            else if (IS_CLASS(*obs, CObservationStereoImages))
            {
                latest_imgs.push(
                    std::dynamic_pointer_cast<CObservationStereoImages>(obs));
                want_to_refresh_view =
                    latest_imgs.size() >= (size_t)N_MIN_IMGS_IN_QUEUE;
            }

            if (want_to_refresh_view)
            {
                // Update the camera image:
                // ----------------------------------
                ASSERT_(!latest_imgs.empty());

                CObservationStereoImages::Ptr latest_img = latest_imgs.front();
                latest_imgs.pop();

                // In order not to load too much the opengl thread, load the
                // JPEG external images in this working thread:
                bool load_ok = false;
                try
                {
                    if (latest_img->imageLeft.loadFromFile(
                            latest_img->imageLeft
                                .getExternalStorageFileAbsolutePath()))
                    {
                        view_im1->setImageView(latest_img->imageLeft);
                        load_ok = true;
                    }
                }
                catch (...)
                {
                }

                if (!load_ok)
                    cerr << "**WARNING**: Image seem damaged: "
                         << latest_img->imageLeft.getExternalStorageFile()
                         << endl;

                // Put malaga map image:
                {
                    static CImage org_map;
                    static bool   first = true;
                    if (first)
                    {
                        first = false;
                        org_map.loadFromFile(MY_SOURCE_DIR "/malaga_sat.png");
                    }

                    CImage mod_map = org_map;

                    TPixelCoordf px;
                    gps_coords2map_px(last_coords, px);

                    // cout << "px: " << px << endl;
                    for (int i = 0; i < 5; i++)
                        mod_map.rectangle(
                            px.x - 10 - i, px.y - 10 - i, px.x + 10 + i,
                            px.y + 10 + i, TColor(1, 1, 0), 1);

                    TPixelCoordf px1, px2;
                    const float  W = 200.0f;
                    const float  H = 100.0f;
                    px1.x          = std::max(0.f, px.x - W);
                    px1.y          = std::max(0.f, px.y - H);
                    px2.x = std::min((float)org_map.getWidth() - 2, px.x + W);
                    px2.y = std::min((float)org_map.getHeight() - 2, px.y + H);

                    CImage aux;
                    mod_map.extract_patch(
                        aux, px1.x, px1.y, px2.x - px1.x + 1,
                        px2.y - px1.y + 1);

                    view_map->setImageView(aux);
                }

                // Update text labels:
                // ----------------------------------
                mrpt::opengl::TFontParams fp;

                scene.getViewport()->addTextMessage(
                    0.42, -13,
                    std::string("Time: ") + mrpt::system::dateTimeLocalToString(
                                                latest_img->timestamp),
                    0 /*id*/, fp);

                //                 fbo.addTextMessage(0.42,-28,format("Timestamp:
                //%f",mrpt::system::timestampToDouble(latest_img->timestamp)),
                // TColorf(1, 1, 1), 1, MRPT_GLUT_BITMAP_HELVETICA_12);

                // fbo.addTextMessage(2,2,string("Lat:
                // ")+last_coords.lat.getAsString(), TColorf(1,1,0), 2,
                // MRPT_GLUT_BITMAP_HELVETICA_12 );
                // fbo.addTextMessage(2,2+17,string("Lon:
                // ")+last_coords.lon.getAsString(), TColorf(1,1,0), 3,
                // MRPT_GLUT_BITMAP_HELVETICA_12 );

                // Update laser scans:
                // --------------------------------
                for (map<string, opengl::CPlanarLaserScan::Ptr>::iterator it =
                         gl_laser_scans.begin();
                     it != gl_laser_scans.end(); ++it)
                {
                    TMapTime2Lasers& mt2l = lstLaserScansByName[it->first];

                    TMapTime2Lasers::iterator itL =
                        mt2l.lower_bound(latest_img->timestamp);

                    if (itL != mt2l.end()) it->second->setScan(*itL->second);
                }

                // Update the 3D point cloud:
                // ----------------------------------
                static int decimate_refresh_points = 0;

                if (++decimate_refresh_points > 20)
                {
                    decimate_refresh_points = 0;

                    for (TMapTime2Lasers::const_iterator it =
                             lstLaserScans.begin();
                         it != lstLaserScans.end(); ++it)
                    {
                        bool    valid;
                        CPose3D P;
                        vehPath.interpolate(it->first, P, valid);
                        if (!valid) continue;

                        ptsMap.insertObservationPtr(it->second, &P);
                    }

                    gl_pointcloud->clear();
                    ptsMap.getAs3DObject(gl_pointcloud);
                    ptsMap.clear();  // Free memory.

                    // Delete old scans:
                    // ----------------------------

                    // Delete from master map:
                    auto diff =
                        latest_img->timestamp -
                        mrpt::system::time_tToTimestamp(TIME_WINDOW_SECS);

                    const mrpt::system::TTimeStamp old_tim =
                        mrpt::system::time_tToTimestamp(
                            std::chrono::duration<double>(diff).count());

                    lstLaserScans.erase(
                        lstLaserScans.begin(),
                        lstLaserScans.lower_bound(old_tim));

                    // and secondary maps by names:
                    for (map<string, TMapTime2Lasers>::iterator it =
                             lstLaserScansByName.begin();
                         it != lstLaserScansByName.end(); ++it)
                        it->second.erase(
                            it->second.begin(),
                            it->second.lower_bound(old_tim));
                }

                // Interpolate the vehicle pose to that timestamp of the camera:
                // ----------------------------------
                bool    valid;
                CPose3D cur_veh_pose;
                vehPath.interpolate(latest_img->timestamp, cur_veh_pose, valid);

                if (valid)
                {
                    gl_veh->setPose(cur_veh_pose);

                    // grid:
                    gl_grid->setPlaneLimits(
                        int(cur_veh_pose.x() / GRID_XY_FREQ) * GRID_XY_FREQ -
                            100,
                        int(cur_veh_pose.x() / GRID_XY_FREQ) * GRID_XY_FREQ +
                            100,
                        int(cur_veh_pose.y() / GRID_XY_FREQ) * GRID_XY_FREQ -
                            100,
                        int(cur_veh_pose.y() / GRID_XY_FREQ) * GRID_XY_FREQ +
                            100);
                    gl_grid->setPlaneZcoord(cur_veh_pose.z() - 5);

                    // Update the camera:
                    camera.setPointingAt(
                        cur_veh_pose.x(), cur_veh_pose.y(), cur_veh_pose.z());
                }
                else
                {
                    cout << "warning: Invalid interpolated pose...\n";
                }

                // Render image:
                CImage render_im;
                fbo.getFrame(scene, render_im);
                win2.showImage(render_im);

                if (GRAB_VIDEO)
                { render_im.saveToFile(format(img_filenames, img_idx++)); }
            }

        };  // end while

        cout << "#imgs: " << img_idx << endl;

        cout << "\nAll done, close the window to quit.\n";

        win2.waitForKey();
        return 0;
    }
    catch (exception& e)
    {
        cerr << "EXCEPTION: " << e.what() << endl;
        return -1;
    }
    catch (...)
    {
        cerr << "Untyped excepcion!!";
        return -1;
    }
}

void gps_coords2map_px(const topography::TGeodeticCoords& gps, TPixelCoordf& px)
{
    // LON: -4 28' 34.596'' =
    static const double lon0 = -4.476276666666;
    // LAT: 36 42' 54.564'' =
    static const double lat0 = 36.71515666666;
    // --> px(312,238)
    static const double x0 = 312, y0 = 238;

    // LON: -4 28' 46.896'' = -4.47969333333   **** (no)
    // LAT: 36 43' 28.074'' = 36.7244650000
    // --> px(309,69)

    // LON: -4.42922833333333
    static const double lon1 = -4.42922833333333;
    // LAT: 36.7180083333333
    static const double lat1 = 36.7180083333333;
    // --> px()
    static const double x1 = 1028, y1 = 186;

    px.x = x1 + (gps.lon.decimal_value - lon1) * (x0 - x1) / (lon0 - lon1);
    px.y = y1 + (gps.lat.decimal_value - lat1) * (y0 - y1) / (lat0 - lat1);
}
