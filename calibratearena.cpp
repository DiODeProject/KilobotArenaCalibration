#include "calibratearena.h"
#include <QImage>
#include <QDebug>
#include <QThread>
#include <QDir>
#include <QSettings>
#include <QFileDialog>


/*!
 * \brief The stitchThread class
 * As the stitching can hang, we run it in a seperate thread
 */
class stitchThread : public QThread
{
public:
    // the data we need to run the stitcher
    vector <Mat> cameraCalibrationImages;
    vector<detail::ImageFeatures> features;
    vector<detail::MatchesInfo> pairwise_matches;

    // the stitcher output
    Mat finalImage;

    // reprojection details to save...
    vector < Mat > Ks;
    vector < Mat > Rs;


private:
    /*!
     * \brief run
     * The execution method for the thread, performing the stitching process
     */
    void run() {

        // Camera estimation
        detail::HomographyBasedEstimator estimator;
        vector<detail::CameraParams> cameras;
        estimator(features, pairwise_matches, cameras);

        for (size_t i = 0; i < cameras.size(); ++i)
         {
             Mat R;
             cameras[i].R.convertTo(R, CV_32F);
             cameras[i].R = R;
         }

        // Refine projection
        Ptr<detail::BundleAdjusterBase> adjuster;
        adjuster = makePtr<detail::BundleAdjusterReproj>();
        adjuster->setConfThresh(0.6f);
        Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
        refine_mask(0,0) = 1;
        refine_mask(0,1) = 1;
        refine_mask(0,2) = 1;
        refine_mask(1,1) = 1;
        refine_mask(1,2) = 1;
        adjuster->setRefinementMask(refine_mask);
        (*adjuster)(features, pairwise_matches, cameras);

        // Find median focal length
        vector<double> focals;
        for (size_t i = 0; i < cameras.size(); ++i)
        {
            focals.push_back(cameras[i].focal);
        }

        sort(focals.begin(), focals.end());
        float warped_image_scale;
        if (focals.size() % 2 == 1)
            warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
        else
            warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

        vector<Mat> rmats;
        for (size_t i = 0; i < cameras.size(); ++i)
            rmats.push_back(cameras[i].R.clone());
        detail::waveCorrect(rmats, detail::WAVE_CORRECT_HORIZ);
        for (size_t i = 0; i < cameras.size(); ++i)
            cameras[i].R = rmats[i];

        vector<UMat> masks(cameraCalibrationImages.size());
        vector<UMat> masks_warped(cameraCalibrationImages.size());
        vector<UMat> images_warped(cameraCalibrationImages.size());
        vector<Point> corners(cameraCalibrationImages.size());
        vector<Size> sizes(cameraCalibrationImages.size());

        // Prepare images masks
        for (int i = 0; i < cameraCalibrationImages.size(); ++i)
        {
          masks[i].create(cameraCalibrationImages[i].size(), CV_8U);
          masks[i].setTo(Scalar::all(255));
        }

        Ptr<WarperCreator> warper_creator;
        warper_creator = makePtr<cv::PlaneWarper>();
        Ptr<detail::RotationWarper> warper = warper_creator->create(3000.0f);


        for (int i = 0; i < cameraCalibrationImages.size(); ++i) {
            Mat_<float> K;
            cameras[i].K().convertTo(K, CV_32F);
            corners[i] = warper->warp(cameraCalibrationImages[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
            sizes[i] = images_warped[i].size();
            warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
        }

        // calculate to compensate for exposure
        Ptr<detail::ExposureCompensator> compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::GAIN);
        compensator->feed(corners, images_warped, masks_warped);

        // apply compensation
        for (int i = 0; i < cameraCalibrationImages.size(); ++i) {
            compensator->apply(i, corners[i], images_warped[i], masks_warped[i]);
        }

        // feather the images together
        Ptr<detail::Blender> blender;
        blender = detail::Blender::createDefault(detail::Blender::FEATHER, false);
        blender->prepare(corners, sizes);
        vector <Mat> images_warped_s(images_warped.size());
        for (int i = 0; i < cameraCalibrationImages.size(); ++i)
        {
            images_warped[i].convertTo(images_warped_s[i], CV_16S);
            blender->feed(images_warped_s[i], masks_warped[i], corners[i]);
        }

        Mat result, result_mask;
        blender->blend(result, result_mask);

        // convert (not sure what this does, but is necessary apparantly)
        result.convertTo(result, (result.type() / 8) * 8);

        cv::resize(result, finalImage,Size(1536,1536));

        // send back the necessary transformation Matrices
        for (uint i = 0; i < cameras.size(); ++i) {
            this->Ks.push_back(cameras[i].K());
            this->Rs.push_back(cameras[i].R);
        }
    }
};


CalibrateArena::CalibrateArena(QPoint smallImageSize, QObject *parent) : QObject(parent)
{
    this->smallImageSize = smallImageSize;
}

CalibrateArena::~CalibrateArena()
{
    // clean up memory
    if (this->thread) {
        delete this->thread;
    }
}

void CalibrateArena::setCalibrationImages(vector<Mat> calImgs)
{
    this->cameraCalibrationImages = calImgs;

    // create the small images and populate them from the big images
    vector <Mat *> imgsSmall;
    for (uint i = 0; i < this->cameraCalibrationImages.size(); ++i) {
        imgsSmall.push_back(new Mat);
        cv::resize(this->cameraCalibrationImages[i], *imgsSmall[i], Size(this->smallImageSize.x(),this->smallImageSize.y()));
    }

    for (uint i = 0; i < imgsSmall.size(); ++i)
    {
        cv::cvtColor(*imgsSmall[i], *imgsSmall[i], CV_BGR2RGB);
        IplImage imageIpl = *imgsSmall[i];
        QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);
        QPixmap pix = QPixmap::fromImage(qimg);
        // send the pixmap to the respective QLabel
        if (i == 0) {
            emit setImage0(pix);
        } else if (i == 1) {
            emit setImage1(pix);
        } else if (i == 2) {
            emit setImage2(pix);
        } else if (i == 3) {
            emit setImage3(pix);
        }
    }

    // delete the small images
    for (uint i = 0; i < imgsSmall.size(); ++i) {
        delete imgsSmall[i];
    }

    emit errorMessage("Images loaded");

}

void CalibrateArena::setFeatureFinderThreshold(int val)
{
    this->featureFinderThreshold = val;
}

void CalibrateArena::setMatcherThreshold(int val)
{
    this->matcherThreshold = float(val)/100.0f;
}


void CalibrateArena::extractFeatures()
{

    // we need at least one image to work on
    if (this->cameraCalibrationImages.size() != 4)
    {
        emit errorMessage("Incorrect calibration image number");
        return;
    }

    // all images must be the same size
    for (uint i = 1; i < this->cameraCalibrationImages.size(); ++i) {
        if (this->cameraCalibrationImages[i].size() != this->cameraCalibrationImages[0].size()) {
            emit errorMessage("Not all calibration images are the same size");
            return;
        }
    }

    // reset the match flag
    this->goodMatches = false;

    // Set up the small images for visualisation
    float smallImXRatio = float(this->smallImageSize.x())/cameraCalibrationImages[0].size().width;
    float smallImYRatio = float(this->smallImageSize.y())/cameraCalibrationImages[0].size().height;

    // create the small images and populate them from the big images
    vector <Mat *> imgsSmall;
    for (uint i = 0; i < this->cameraCalibrationImages.size(); ++i) {
        imgsSmall.push_back(new Mat);
        cv::resize(this->cameraCalibrationImages[i], *imgsSmall[i], Size(this->smallImageSize.x(),this->smallImageSize.y()));
    }

    // extract information for feature finding, and set up the vectors for each image's features
    // clear old features
    this->features.clear();
    this->features.resize(this->cameraCalibrationImages.size());
    vector<Size> full_img_sizes(this->cameraCalibrationImages.size());
    for (uint i = 0; i < this->cameraCalibrationImages.size(); ++i) {
        full_img_sizes[i] = this->cameraCalibrationImages[i].size();
    }

    // create the feature finder
    Ptr<detail::FeaturesFinder> finder;
    finder = makePtr<detail::SurfFeaturesFinder>(this->featureFinderThreshold);

    // ROI finder (SURF)
    for (uint i = 0; i < this->cameraCalibrationImages.size(); ++i) {
        (*finder)(this->cameraCalibrationImages[i], features[i]);
        features[i].img_idx = i;
        // replace with feedback
        //qDebug() << "Features in image #" << i+1 << ": " << features[i].keypoints.size();
        for (int j = 0; j < features[i].keypoints.size(); ++j) {
            circle(*imgsSmall[i], Size(smallImXRatio * features[i].keypoints[j].pt.x,smallImYRatio * features[i].keypoints[j].pt.y), 1.0f, Scalar(100, 0, 0));
        }
    }

    // clear old matches
    this->pairwise_matches.clear();

    // Pairwise matcher
    detail::BestOf2NearestMatcher matcher(false, this->matcherThreshold);
    matcher(features, pairwise_matches);
    matcher.collectGarbage();

    vector<int> indices = detail::leaveBiggestComponent(features, pairwise_matches, 0.5f);

    // for use with Qt::GlobalColor
    int c = 6;

    for (uint i = 0; i < pairwise_matches.size(); ++i) {
        int src_ind = pairwise_matches[i].src_img_idx;
        int dst_ind = pairwise_matches[i].dst_img_idx;
        vector<DMatch> matches = pairwise_matches[i].matches;
        if (src_ind < dst_ind) { // only show one-way matches
            QColor col = (Qt::GlobalColor)(++c);
            for (uint j = 0; j < matches.size(); ++j) {
                DMatch match = matches[j];
                circle(*imgsSmall[src_ind], Size(smallImXRatio * features[src_ind].keypoints[match.queryIdx].pt.x,smallImYRatio * features[src_ind].keypoints[match.queryIdx].pt.y), 3.0f, Scalar(col.red(),col.green(),col.blue()));
                circle(*imgsSmall[dst_ind], Size(smallImXRatio * features[dst_ind].keypoints[match.trainIdx].pt.x,smallImYRatio * features[dst_ind].keypoints[match.trainIdx].pt.y), 3.0f, Scalar(col.red(),col.green(),col.blue()));
            }
        }
    }


    // display the images
    for (uint i = 0; i < imgsSmall.size(); ++i)
    {
        cv::cvtColor(*imgsSmall[i], *imgsSmall[i], CV_BGR2RGB);
        IplImage imageIpl = *imgsSmall[i];
        QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);
        QPixmap pix = QPixmap::fromImage(qimg);
        // send the pixmap to the respective QLabel
        if (i == 0) {
            emit setFeaturesImage0(pix);
        } else if (i == 1) {
            emit setFeaturesImage1(pix);
        } else if (i == 2) {
            emit setFeaturesImage2(pix);
        } else if (i == 3) {
            emit setFeaturesImage3(pix);
        }
    }

    // check how many images we have in the matched set (must be all for success (i.e. 4))
    if (indices.size() < 4) {
        emit errorMessage("Cannot match all the images: try reducing the feature and/or match thresholds");
        // delete the small images
        for (uint i = 0; i < imgsSmall.size(); ++i) {
            delete imgsSmall[i];
        }
        return;
    }


    // delete the small images
    for (uint i = 0; i < imgsSmall.size(); ++i) {
        delete imgsSmall[i];
    }

    // success! Set the match flag
    this->goodMatches = true;
    emit errorMessage("Features extracted successfully");

}

void CalibrateArena::stitchImages()
{

    // check that we have features
    if (!this->goodMatches) {
        emit errorMessage("No good matches, please repeat feature extraction");
        return;
    }

    // since this process can hang, we use a seperate thread, and allow the user to abort the process by re-calling this
    // method
    if (thread != NULL && thread->isRunning()) {

        thread->terminate();
        thread->wait(10);
        if (thread->isRunning()) {
            emit errorMessage("Couldn't end stitcher thread");
        } else {
            emit errorMessage("Stitcher thread terminated");
            QPushButton * src = qobject_cast < QPushButton * > (this->sender());
            if (src) {
                src->setText("Stitch images");
            }
        }
        return;
    }

    // no stitcher running, so launch a new one (create if necessary
    if (thread == NULL) {
        thread = new stitchThread;
    }

    thread->cameraCalibrationImages = this->cameraCalibrationImages;
    thread->pairwise_matches = this->pairwise_matches;
    thread->features = this->features;
    connect(this->thread, SIGNAL(finished()), this, SLOT(stitcherFinished()));
    thread->start();

    QPushButton * src = qobject_cast < QPushButton * > (this->sender());
    this->stitchButton = src;
    if (src) {
        src->setText("Abort stitching");
    }

    emit errorMessage("Stitcher thread running...");

}

void CalibrateArena::stitcherFinished()
{
    // safety first!
    if (this->thread != NULL) {

        if (this->thread->finalImage.size().width < 100) {
            return;
        }

        Mat result = this->thread->finalImage;

        // the *2 is an assumption - should always be true...
        cv::resize(result,result,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));
        cv::cvtColor(result, result, CV_BGR2RGB);

        // convert to C header for easier mem ptr addressing
        IplImage imageIpl = result;

        // create a QImage container pointing to the image data
        QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

        // assign to a QPixmap (may copy)
        QPixmap pix = QPixmap::fromImage(qimg);

        emit setStitchedImage(pix);
        emit errorMessage("Stitching complete");
        this->stitchButton->setText("Stitch images");
    }
}

void CalibrateArena::pointSelected(QPoint point)
{

    if (arenaCorners.size() < 4)
    {
        arenaCorners.push_back(point);
    }

    // draw points
    if (this->thread != NULL && !this->thread->isRunning()) {

        if (this->thread->finalImage.size().width < 100) {
            return;
        }

        Mat result = this->thread->finalImage;

        // the *2 is an assumption - should always be true...
        cv::resize(result,result,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));

        // add points
        for (uint i = 0; i < this->arenaCorners.size(); ++i) {
            circle(result, Size(arenaCorners[i].x(),arenaCorners[i].y()), 3.0f, Scalar(0,255,0));
        }

        cv::cvtColor(result, result, CV_BGR2RGB);

        // convert to C header for easier mem ptr addressing
        IplImage imageIpl = result;

        // create a QImage container pointing to the image data
        QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

        // assign to a QPixmap (may copy)
        QPixmap pix = QPixmap::fromImage(qimg);

        emit setStitchedImage(pix);

    }

}

void CalibrateArena::squareArena()
{

    // do we have 4 points?
    if (this->arenaCorners.size() < 4) {
        emit errorMessage("4 points needed to square - select them on the stitched image in the previous tab");
        return;
    }


    if (this->thread != NULL && !this->thread->isRunning()) {

        if (this->thread->finalImage.size().width < 100) {
            return;
        }

        // square the image

        Point2f inputQuad[4];
        Point2f outputQuad[4];

        for( size_t i = 0; i < arenaCorners.size(); i++ )
        {
            // convert co-ordinates
            Point center( float(arenaCorners[i].x())*float(this->thread->finalImage.size().width)/float(this->smallImageSize.x()*2) , float(arenaCorners[i].y())*float(this->thread->finalImage.size().height)/float(this->smallImageSize.y()*2) );

            if (center.x > this->thread->finalImage.size().width/2.0 && center.y > this->thread->finalImage.size().height/2) {
                inputQuad[3] = center;
            }
            if (center.x > this->thread->finalImage.size().width/2.0 && center.y < this->thread->finalImage.size().height/2) {
                inputQuad[1] = center;
            }
            if (center.x < this->thread->finalImage.size().width/2.0 && center.y > this->thread->finalImage.size().height/2) {
                inputQuad[2] = center;
            }
            if (center.x < this->thread->finalImage.size().width/2.0 && center.y < this->thread->finalImage.size().height/2) {
                inputQuad[0] = center;
            }
        }

        outputQuad[0] = Point(0,0);
        outputQuad[1] = Point(2000,0);
        outputQuad[2] = Point(0,2000);
        outputQuad[3] = Point(2000,2000);

        Mat M = getPerspectiveTransform(inputQuad,outputQuad);
        warpPerspective(this->thread->finalImage, this->fullSizeFinalIm, M, Size(2000,2000));

        cv::cvtColor(this->fullSizeFinalIm, this->fullSizeFinalIm, CV_BGR2RGB);

        // set label
        Mat shrunkIm = this->fullSizeFinalIm;

        cv::resize(this->fullSizeFinalIm, shrunkIm, Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));

        // convert to C header for easier mem ptr addressing
        IplImage imageIpl = shrunkIm;

        // create a QImage container pointing to the image data
        QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

        // assign to a QPixmap (may copy)
        QPixmap pix = QPixmap::fromImage(qimg);

        emit setSquaredImage(pix);
        emit errorMessage("Squaring complete");

    }

}

void CalibrateArena::resetPoint()
{
    if (arenaCorners.size() > 0) {
        arenaCorners.pop_back();
    }

    // draw points
    if (this->thread != NULL && !this->thread->isRunning()) {

        if (this->thread->finalImage.size().width < 100) {
            return;
        }

        Mat result = this->thread->finalImage;

        // the *2 is an assumption - should always be true...
        cv::resize(result,result,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));

        // add points
        for (uint i = 0; i < this->arenaCorners.size(); ++i) {
            circle(result, Size(arenaCorners[i].x(),arenaCorners[i].y()), 3.0f, Scalar(0,255,0));
        }

        cv::cvtColor(result, result, CV_BGR2RGB);

        // convert to C header for easier mem ptr addressing
        IplImage imageIpl = result;

        // create a QImage container pointing to the image data
        QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

        // assign to a QPixmap (may copy)
        QPixmap pix = QPixmap::fromImage(qimg);

        emit setStitchedImage(pix);

    }
}

void CalibrateArena::saveCalibration()
{

    if (this->thread != NULL && !this->thread->isRunning()) {

        if (this->thread->finalImage.size().width < 100) {
            emit errorMessage("No valid stitched image generated");
            return;
        }

        if (arenaCorners.size() < 4) {
            emit errorMessage("Arena corners for squaring not set");
            return;
        }

        // Ok, all seems good, lets save the data:

        QSettings settings;
        QString lastDir = settings.value("lastDirOut", QDir::homePath()).toString();
        QString fileName = QFileDialog::getSaveFileName((QWidget *) sender(), tr("Save Calibration"), lastDir, tr("XML files (*.xml);; All files (*)"));

        if (fileName.isEmpty()) {
            emit errorMessage("No save file given");
            return;
        }

        // save the data
        FileStorage fs(fileName.toStdString(),FileStorage::WRITE);

        Point2f inputQuad[4];

        for (uint i = 0; i < this->arenaCorners.size(); ++i)
        {
            // convert co-ordinates
            Point center( float(arenaCorners[i].x())*float(this->thread->finalImage.size().width)/float(this->smallImageSize.x()*2) , float(arenaCorners[i].y())*float(this->thread->finalImage.size().height)/float(this->smallImageSize.y()*2) );

            if (center.x > this->thread->finalImage.size().width/2.0 && center.y > this->thread->finalImage.size().height/2) {
                inputQuad[3] = center;
            }
            if (center.x > this->thread->finalImage.size().width/2.0 && center.y < this->thread->finalImage.size().height/2) {
                inputQuad[1] = center;
            }
            if (center.x < this->thread->finalImage.size().width/2.0 && center.y > this->thread->finalImage.size().height/2) {
                inputQuad[2] = center;
            }
            if (center.x < this->thread->finalImage.size().width/2.0 && center.y < this->thread->finalImage.size().height/2) {
                inputQuad[0] = center;
            }
        }

        fs << "corner1" << inputQuad[0];
        fs << "corner2" << inputQuad[1];
        fs << "corner3" << inputQuad[2];
        fs << "corner4" << inputQuad[3];

        fs << "R" << this->thread->Rs;
        fs << "K" << this->thread->Ks;

        QDir lastDirectory (fileName);
        lastDirectory.cdUp();
        settings.setValue ("lastDirOut", lastDirectory.absolutePath());

    }

}

void CalibrateArena::zoomMove(QPoint pos)
{

    Size sz(float(pos.x())/float(this->smallImageSize.x()*2)*float(this->fullSizeFinalIm.size().width) , float(pos.y())/float(this->smallImageSize.y()*2)*float(this->fullSizeFinalIm.size().height));

    if (sz.width < this->smallImageSize.x()) sz.width = this->smallImageSize.x();
    if (sz.width > this->fullSizeFinalIm.size().width-this->smallImageSize.x()-1) sz.width = this->fullSizeFinalIm.size().width-this->smallImageSize.x()-1;
    if (sz.height < this->smallImageSize.y()) sz.height = this->smallImageSize.y();
    if (sz.height > this->fullSizeFinalIm.size().height-this->smallImageSize.y()-1) sz.height = this->fullSizeFinalIm.size().height-this->smallImageSize.y()-1;

    // set label
    Mat shrunkIm = this->fullSizeFinalIm(Rect(sz.width-this->smallImageSize.x(),sz.height-this->smallImageSize.y(),this->smallImageSize.x()*2,this->smallImageSize.y()*2));

    Mat s2;
    shrunkIm.copyTo(s2);

    // convert to C header for easier mem ptr addressing
    IplImage imageIpl = s2;

    // create a QImage container pointing to the image data
    QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

    // assign to a QPixmap (may copy)
    QPixmap pix = QPixmap::fromImage(qimg);

    emit setSquaredImage(pix);
}

void CalibrateArena::zoomMoveDone()
{
    // set label
    Mat shrunkIm;

    cv::resize(this->fullSizeFinalIm, shrunkIm, Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));

    // convert to C header for easier mem ptr addressing
    IplImage imageIpl = shrunkIm;

    // create a QImage container pointing to the image data
    QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

    // assign to a QPixmap (may copy)
    QPixmap pix = QPixmap::fromImage(qimg);

    emit setSquaredImage(pix);
}
