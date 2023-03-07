/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// -----------------------------------------------------------------------------------------------------------------------
// For this example, the rawlog file must contain both laser data and stereo
// images (only the left one will be considered)
// It may be used with single image observations -> just employ
// "CObservationImage::Ptr" instead of "CObservationStereoImages::Ptr"
// and access to the contained "image" instead of "imageLeft".
// -----------------------------------------------------------------------------------------------------------------------

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/vision/pinhole.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::img;
using namespace std;

std::string datasetFile;

std::string LIDAR_SENSOR_LABEL_FRONT = "LASER2";
std::string LIDAR_SENSOR_LABEL_DOWN  = "HOKUYO1";
std::string LIDAR_SENSOR_LABEL_RIGHT = "HOKUYO2";
std::string LIDAR_SENSOR_LABEL_LEFT  = "HOKUYO3";

// Fixed rotation from +X forward, +Z up == > +X right, +Y down:
const auto cameraAxisConvention =
    CPose3D::FromYawPitchRoll(-90.0_deg, 0.0, -90.0_deg);

// ------------------------------------------------------
//                  TestGeometry3D
// ------------------------------------------------------
void TestLaser2Imgs()
{
    // Set your rawlog file name
    ASSERT_FILE_EXISTS_(datasetFile);

    size_t           rawlogEntry = 0;
    CDisplayWindow3D win("Lidar-to-image projection example - MRPT", 1024, 800);

    mrpt::opengl::Viewport::Ptr viewImg, view3D;
    auto glFrontLidarPts = mrpt::opengl::CPointCloud::Create();
    auto glDownLidarPts  = mrpt::opengl::CPointCloud::Create();
    auto glLeftLidarPts  = mrpt::opengl::CPointCloud::Create();
    auto glRightLidarPts = mrpt::opengl::CPointCloud::Create();

    auto glCornerFrontLidar = mrpt::opengl::stock_objects::CornerXYZ(0.2f);
    auto glCornerDownLidar  = mrpt::opengl::stock_objects::CornerXYZ(0.2f);
    auto glCornerLeftLidar  = mrpt::opengl::stock_objects::CornerXYZ(0.2f);
    auto glCornerRightLidar = mrpt::opengl::stock_objects::CornerXYZ(0.2f);
    auto glCornerCamera     = mrpt::opengl::stock_objects::CornerXYZ(0.2f);

    auto glCameraFrustrum = mrpt::opengl::CSetOfObjects::Create();
    bool frustrumUpdated  = false;

    win.setCameraAzimuthDeg(220);
    win.setCameraElevationDeg(20);

    {
        auto& scene = win.get3DSceneAndLock();

        view3D = scene->createViewport("main");
        view3D->insert(mrpt::opengl::CAxis::Create(-30, -30, -5, 30, 30, 5));

        glFrontLidarPts->setPointSize(4.0f);
        glFrontLidarPts->setColor_u8(0xff, 0x00, 0x00);
        view3D->insert(glFrontLidarPts);

        glDownLidarPts->setPointSize(4.0f);
        glDownLidarPts->enableColorFromZ();
        view3D->insert(glDownLidarPts);

        glLeftLidarPts->setPointSize(4.0f);
        glLeftLidarPts->enableColorFromZ();
        view3D->insert(glLeftLidarPts);

        glRightLidarPts->setPointSize(4.0f);
        glRightLidarPts->enableColorFromZ();
        view3D->insert(glRightLidarPts);

        view3D->insert(glCornerFrontLidar);
        view3D->insert(glCornerDownLidar);
        view3D->insert(glCornerLeftLidar);
        view3D->insert(glCornerRightLidar);
        view3D->insert(glCornerCamera);

        glCornerFrontLidar->setName(LIDAR_SENSOR_LABEL_FRONT);
        glCornerFrontLidar->enableShowName();

        glCornerDownLidar->setName(LIDAR_SENSOR_LABEL_DOWN);
        glCornerDownLidar->enableShowName();

        glCornerLeftLidar->setName(LIDAR_SENSOR_LABEL_LEFT);
        glCornerLeftLidar->enableShowName();

        glCornerRightLidar->setName(LIDAR_SENSOR_LABEL_RIGHT);
        glCornerRightLidar->enableShowName();

        glCornerCamera->setName("LeftCamera");
        glCornerCamera->enableShowName();

        view3D->insert(glCameraFrustrum);

        viewImg = scene->createViewport("img");
        viewImg->setViewportPosition(0, 0, 0.5, 0.5);
        viewImg->setTransparent(true);

        win.unlockAccess3DScene();
    }

    // Set relative path for externally-stored images in rawlogs:
    string externalImgsDir = CRawlog::detectImagesDirectory(datasetFile);
    ASSERT_FILE_EXISTS_(externalImgsDir);
    CImage::setImagesPathBase(externalImgsDir);  // Set it.

    std::cout << "Using dataset           : " << datasetFile << "\n";
    std::cout << "With external images in : " << externalImgsDir << "\n";

    mrpt::io::CFileGZInputStream rawlogFile(datasetFile);
    auto arch = mrpt::serialization::archiveFrom(rawlogFile);

    CObservationStereoImages::Ptr lastImgs;
    CObservation2DRangeScan::Ptr  lastFrontLidar, lastDownLidar, lastLeftLidar,
        lastRightLidar;

    for (;;)
    {
        if (os::kbhit())
        {
            char c = os::getch();
            if (c == 27) break;
        }

        CActionCollection::Ptr action;
        CSensoryFrame::Ptr     sf;
        CObservation::Ptr      obs;

        // Load observations from the rawlog:
        // --------------------------------------------------
        if (!CRawlog::getActionObservationPairOrObservation(
                arch, action, sf, obs, rawlogEntry))
            break;  // file EOF

        if (!obs) continue;

        bool anyNew = false;

        // CAMERA?
        if (auto im = std::dynamic_pointer_cast<CObservationStereoImages>(obs);
            im)
        {
            lastImgs = im;
            anyNew   = true;
            std::cout << "New stereo image processed, timestamp = "
                      << mrpt::system::dateTimeLocalToString(
                             lastImgs->timestamp)
                      << "\n";
        }

        // 2D lidar?
        if (auto oLidar =
                std::dynamic_pointer_cast<CObservation2DRangeScan>(obs);
            oLidar && oLidar->sensorLabel == LIDAR_SENSOR_LABEL_FRONT)
        {
            anyNew         = true;
            lastFrontLidar = oLidar;
        }

        if (auto oLidar =
                std::dynamic_pointer_cast<CObservation2DRangeScan>(obs);
            oLidar && oLidar->sensorLabel == LIDAR_SENSOR_LABEL_DOWN)
        {
            anyNew        = true;
            lastDownLidar = oLidar;
        }

        if (auto oLidar =
                std::dynamic_pointer_cast<CObservation2DRangeScan>(obs);
            oLidar && oLidar->sensorLabel == LIDAR_SENSOR_LABEL_RIGHT)
        {
            anyNew         = true;
            lastRightLidar = oLidar;
        }
        if (auto oLidar =
                std::dynamic_pointer_cast<CObservation2DRangeScan>(obs);
            oLidar && oLidar->sensorLabel == LIDAR_SENSOR_LABEL_LEFT)
        {
            anyNew        = true;
            lastLeftLidar = oLidar;
        }

        if (!lastImgs || !lastFrontLidar || !lastDownLidar) continue;
        if (!anyNew) continue;

        // Get Camera Pose (B) (CPose3D)
        auto orgCamPose = lastImgs->cameraPose.asTPose();

        // for malaga urban dataset paper, there was a typo in the camera Z
        // coordinate. See table 2:
        // https://ingmec.ual.es/~jlblanco/papers/blanco2013malaga_urban_dataset_IJRR_draft.pdf
        if (std::abs(orgCamPose.z - 0.273) < 0.01)
            orgCamPose.z = 0.057;  // fix it

        const CPose3D camPose = CPose3D(orgCamPose) + cameraAxisConvention;

        // Get Calibration matrix (K)
        // make a deep copy of the image since we are modifying it later on:
        CImage         image     = lastImgs->imageLeft.makeDeepCopy();
        const TCamera& camParams = lastImgs->leftCamera;

        // Get Laser Pose (A) (CPose3D)
        const CPose3D frontLaserPose = lastFrontLidar->sensorPose;

        // Get 3D Point relative to the Laser coordinate Frame (P1) (CPoint3D)
        CSimplePointsMap frontLidarPoints;
        frontLidarPoints.insertionOptions.minDistBetweenLaserPoints = 0;
        lastFrontLidar->insertObservationInto(frontLidarPoints);

        CSimplePointsMap downLidarPoints;
        downLidarPoints.insertionOptions.minDistBetweenLaserPoints = 0;
        lastDownLidar->insertObservationInto(downLidarPoints);

        CSimplePointsMap leftLidarPoints;
        leftLidarPoints.insertionOptions.minDistBetweenLaserPoints = 0;
        if (lastLeftLidar)
        {
            lastLeftLidar->insertObservationInto(leftLidarPoints);
            glCornerLeftLidar->setPose(lastLeftLidar->sensorPose);
        }

        CSimplePointsMap rightLidarPoints;
        rightLidarPoints.insertionOptions.minDistBetweenLaserPoints = 0;
        if (lastRightLidar)
        {
            lastRightLidar->insertObservationInto(rightLidarPoints);
            glCornerRightLidar->setPose(lastRightLidar->sensorPose);
        }

        // Get the points into the map
        unsigned int imgW = camParams.ncols;
        unsigned int imgH = camParams.nrows;

        ASSERT_EQUAL_(image.getWidth(), imgW);
        ASSERT_EQUAL_(image.getHeight(), imgH);

        // Project lidar pts into the image plane:
        std::vector<mrpt::math::TPoint2D> imgPts;

        const auto lambdaProject = [&](const CSimplePointsMap&     pts,
                                       const mrpt::poses::CPose3D& cameraPose,
                                       const mrpt::img::TCamera&   camCalib,
                                       const mrpt::img::TColor&    color) {
            const auto&  X    = pts.getPointsBufferRef_x();
            const auto&  Y    = pts.getPointsBufferRef_y();
            const auto&  Z    = pts.getPointsBufferRef_z();
            const size_t nPts = X.size();

            // project into image pixels, including possible distorsion
            // if images are not rectified.
            // Pimg = (kx,ky,k)^T = K(I|0)*P2

            std::vector<mrpt::math::TPoint3D> pts3D;
            for (size_t i = 0; i < nPts; i++)
                pts3D.emplace_back(X[i], Y[i], Z[i]);

            std::vector<mrpt::img::TPixelCoordf> projectedPoints;

            mrpt::vision::pinhole::projectPoints_with_distortion(
                pts3D, cameraPose, camCalib.intrinsicParams,
                camCalib.getDistortionParamsAsVector(), projectedPoints);

            for (const auto& imgPt : projectedPoints)
            {
                if (imgPt.x < 0 || imgPt.x >= imgW || imgPt.y < 0 ||
                    imgPt.y >= imgH)
                    continue;

                image.filledRectangle(
                    imgPt.x - 2, imgPt.y - 2, imgPt.x + 2, imgPt.y + 2, color);
            }
        };  // end lambda

        lambdaProject(frontLidarPoints, camPose, camParams, TColor::red());
        lambdaProject(downLidarPoints, camPose, camParams, TColor::blue());

        // show timestamp:
        win.addTextMessage(
            5, -20,
            mrpt::system::dateTimeLocalToString(lastFrontLidar->timestamp));

        // Update view:
        {
            win.get3DSceneAndLock();

            glCornerFrontLidar->setPose(frontLaserPose);
            glCornerCamera->setPose(camPose);

            glFrontLidarPts->loadFromPointsMap(&frontLidarPoints);

            glDownLidarPts->loadFromPointsMap(&downLidarPoints);
            glLeftLidarPts->loadFromPointsMap(&leftLidarPoints);
            glRightLidarPts->loadFromPointsMap(&rightLidarPoints);

            viewImg->setImageView(std::move(image));

            if (!frustrumUpdated)
            {
                frustrumUpdated = true;
                glCameraFrustrum->insert(
                    mrpt::opengl::CFrustum::Create(camParams));
            }

            win.unlockAccess3DScene();
            win.repaint();
        }

        std::this_thread::sleep_for(1ms);
    };  // end for
}

// ------------------------------------------------------
//                        MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
    try
    {
        if (argc != 2)
        {
            std::cerr << "Usage: " << argv[0] << " /path/to/dataset.rawlog\n";
            return 1;
        }

        datasetFile = std::string(argv[1]);

        TestLaser2Imgs();
        return 0;
    }
    catch (exception& e)
    {
        cerr << "EXCEPTION: " << e.what() << endl;
        return -1;
    }
}
