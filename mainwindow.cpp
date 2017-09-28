#include "mainwindow.h"
#include "ui_mainwindow.h"

// QT includes
#include <QLabel>
#include <QLayout>
#include <QDebug>
#include <QSettings>
#include <QDir>
#include <QFileDialog>

// STL includes
#include <vector>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // ui signal/slots
    connect(ui->matcher_conf_slider,SIGNAL(sliderMoved(int)), this, SLOT(matchConfDoubleConvertor(int)));

    // connect up calibrater signal/slots
    connect(ui->fd_thresh_slider,SIGNAL(sliderMoved(int)),&this->calibrater,SLOT(setFeatureFinderThreshold(int)));
    connect(ui->matcher_conf_slider,SIGNAL(sliderMoved(int)),&this->calibrater,SLOT(setMatcherThreshold(int)));

    connect(ui->load_images, SIGNAL(clicked(bool)), this, SLOT(loadImages()));
    connect(ui->cap_images, SIGNAL(clicked(bool)), this, SLOT(capImages()));
    connect(ui->save_images, SIGNAL(clicked(bool)), this, SLOT(saveImages()));

    connect(ui->extract_features,SIGNAL(clicked(bool)), &this->calibrater, SLOT(extractFeatures()));
    connect(ui->stitch_images,SIGNAL(clicked(bool)), &this->calibrater, SLOT(stitchImages()));
    connect(ui->square_arena,SIGNAL(clicked(bool)), &this->calibrater, SLOT(squareArena()));

    connect(&this->calibrater,SIGNAL(errorMessage(QString)), ui->error_label, SLOT(setText(QString)));

    connect(&calibrater, SIGNAL(setImage0(QPixmap)),ui->im1,SLOT(setPixmap(QPixmap)));
    connect(&calibrater, SIGNAL(setImage1(QPixmap)),ui->im2,SLOT(setPixmap(QPixmap)));
    connect(&calibrater, SIGNAL(setImage2(QPixmap)),ui->im3,SLOT(setPixmap(QPixmap)));
    connect(&calibrater, SIGNAL(setImage3(QPixmap)),ui->im4,SLOT(setPixmap(QPixmap)));

    connect(&calibrater, SIGNAL(setFeaturesImage0(QPixmap)),ui->im1_roi,SLOT(setPixmap(QPixmap)));
    connect(&calibrater, SIGNAL(setFeaturesImage1(QPixmap)),ui->im2_roi,SLOT(setPixmap(QPixmap)));
    connect(&calibrater, SIGNAL(setFeaturesImage2(QPixmap)),ui->im3_roi,SLOT(setPixmap(QPixmap)));
    connect(&calibrater, SIGNAL(setFeaturesImage3(QPixmap)),ui->im4_roi,SLOT(setPixmap(QPixmap)));

    connect(&calibrater, SIGNAL(setStitchedImage(QPixmap)),ui->result,SLOT(setPixmap(QPixmap)));

    connect(&calibrater, SIGNAL(setSquaredImage(QPixmap)),ui->result_final,SLOT(setPixmap(QPixmap)));

    connect(ui->result, SIGNAL(clicked(QPoint)), &this->calibrater, SLOT(pointSelected(QPoint)));
    connect(ui->reset_corners, SIGNAL(clicked(bool)), &this->calibrater, SLOT(resetPoint()));

    connect(ui->result_final, SIGNAL(moving(QPoint)), &this->calibrater, SLOT(zoomMove(QPoint)));
    connect(ui->result_final, SIGNAL(moveDone()), &this->calibrater, SLOT(zoomMoveDone()));
    connect(ui->save_calib, SIGNAL(clicked(bool)), &this->calibrater, SLOT(saveCalibration()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadImages()
{

    QSettings settings;
    QString lastDir = settings.value("lastDir", QDir::homePath()).toString();
    QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Load the Four Calibration Images"), lastDir, tr("Image files (*.jpg *.png);; All files (*)"));

    if (fileNames.size() != 4) {
        ui->error_label->setText("Four calibration images are required");
        return;
    }

    vector <Mat> imgs;

    for (uint i = 0; i < fileNames.size(); ++i) {
        imgs.push_back(imread(fileNames[i].toStdString(), CV_LOAD_IMAGE_COLOR));
        if (!imgs.back().data) {
            ui->error_label->setText("Error loading an image");
            return;
        }
    }

    calibrater.setCalibrationImages(imgs);

    QDir lastDirectory (fileNames[0]);
    lastDirectory.cdUp();
    settings.setValue ("lastDir", lastDirectory.absolutePath());
}

void MainWindow::capImages()
{

    vector <Mat> imgs;

    // try to open and capture images from 4 cameras
    for (uint i = 0; i < 4; ++i) {

        cv::VideoCapture cap(i);

        if (!cap.isOpened()) {
            this->ui->error_label->setText(QString("Only ")+QString::number(i) + QString(" cameras were found, 4 are required for calibration"));
            return;
        } else {
            cap.set(CV_CAP_PROP_FRAME_WIDTH, 2048);
            cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1536);
            Mat frame;
            cap >> frame;
            imgs.push_back(frame);
            cap.release();
        }

    }

    // can't get here if there is a problem, so send the images to the calibrater
    this->calibrater.setCalibrationImages(imgs);

}


void MainWindow::saveImages()
{
    // Get the captured calibration images
    vector <Mat>calibrationimages= this->calibrater.getCameraCalibrationImages();

    // Check if the image were already/correctly captured
    if( calibrationimages.size() == 4 ){

            //Get the images name prefix
            QString name_prefix= ui->lineEdit->text();

            //Get the folder were the captured images will be saved
            QSettings settings;
            QString lastDir = settings.value("lastDir", QDir::homePath()).toString();
            QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                            lastDir,
                                                            QFileDialog::ShowDirsOnly
                                                            | QFileDialog::DontResolveSymlinks);



            //Specify the image compression params
            vector <int> compression_params;
            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            compression_params.push_back(95);

            //Save the captured images one by one
            for (uint i = 0; i < 4; ++i) {
                imwrite(dir.toStdString()+"/"+name_prefix.toStdString()+to_string(i)+".jpg",calibrationimages[i],compression_params);
            }

            //Inform the user that the captured images were correctly saved
            ui->error_label->setText("Calibration images saved!");
    }

    //Inform the user about a possible image capturing error
    else ui->error_label->setText("The calibration images were not captured correctly!");
}






void MainWindow::matchConfDoubleConvertor(int val)
{
    ui->match_conf_label->setText(QString::number(float(val)/100.0f));
}

/*!
 * \brief MainWindow::testStitching
 *
 * A method used to initially test the stitching in OpenCV - will be moved to the
 * calibration class once finalised
 */
void MainWindow::testStitching()
{
/*
    Mat res2;
    cv::cvtColor(result, res2, CV_BGR2GRAY);
    //cv::threshold(res2, res2, 30, 50, cv::THRESH_BINARY + cv::THRESH_OTSU);

   // GaussianBlur(res2,res2,Size(0,0),20);
    vector<Vec3f> circles;

    */

    //HoughCircles(res2,circles,CV_HOUGH_GRADIENT,1/* rez scaling (1 = full rez, 2 = half etc)*/,15 /* Canny threshold*/,120 /*cicle algorithm accuracy*/,10/* circle distance*/,7/* min circle size*/,11/* max circle size*/);

    /*

    qDebug() << "Found" << circles.size() << "circles";

    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         //circle( result, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( result, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
    circle( result, Point(10,20), 5, Scalar(0,255,0), 2, 8, 0 );
    circle( result, Point(500,500), 5, Scalar(0,255,0), 2, 8, 0 );
    circle( result, Point(10,500), 5, Scalar(0,255,0), 2, 8, 0 );
    circle( result, Point(500,20), 5, Scalar(0,255,0), 2, 8, 0 );

    Mat planes[3];
    split(result,planes);
    Mat res3 = planes[1]-0.8*(planes[0]+planes[2]);

    */

    //HoughCircles(res3,circles,CV_HOUGH_GRADIENT,1/* rez scaling (1 = full rez, 2 = half etc)*/,200 /* Canny threshold*/,200 /*cicle algorithm accuracy*/,10/* circle distance*/,0/* min circle size*/,11/* max circle size*/);

    /*

    Point2f inputQuad[4];
    Point2f outputQuad[4];

    outputQuad[0] = Point(0,0);
    outputQuad[1] = Point(ui->result->width(),0);
    outputQuad[2] = Point(0,ui->result->height());
    outputQuad[3] = Point(ui->result->width(),ui->result->height());

    // sort centres
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

        if (center.x > 200 && center.y > 200) {
            inputQuad[3] = center;
        }
        if (center.x > 200 && center.y < 200) {
            inputQuad[1] = center;
        }
        if (center.x < 200 && center.y > 200) {
            inputQuad[2] = center;
        }
        if (center.x < 200 && center.y < 200) {
            inputQuad[0] = center;
        }
    }

    Mat M = getPerspectiveTransform(inputQuad,outputQuad);
    Mat result2;
    warpPerspective(result, result2, M, Size(ui->result->width(),ui->result->height()));

    // goodFeaturesToTrack



//    Canny( res2, res2, 0, 10, 3 );

//    findContours(res2, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

//    drawContours(result,contours,-1,Scalar(255,0,0));

//    vector<vector<Point> > squares;

//    vector<Point> approx;

//    for( size_t i = 0; i < contours.size(); i++ )
//    {
//        // approximate contour with accuracy proportional
//        // to the contour perimeter
//        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.1, true);

//        // square contours should have 4 vertices after approximation
//        // relatively large area (to filter out noisy contours)
//        // and be convex.
//        // Note: absolute value of an area is used because
//        // area may be positive or negative - in accordance with the
//        // contour orientation
//        if( approx.size() == 4 &&
//            fabs(contourArea(Mat(approx))) > 1000 &&
//            isContourConvex(Mat(approx)) )
//        {
//            double maxCosine = 0;

//            for( int j = 2; j < 5; j++ )
//            {
//                // find the maximum cosine of the angle between joint edges
//                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
//                maxCosine = MAX(maxCosine, cosine);
//            }

//            // if cosines of all angles are small
//            // (all angles are ~90 degree) then write quandrange
//            // vertices to resultant sequence
//            //if( maxCosine < 0.9 )
//                squares.push_back(approx);
//        }
//    }

//    for( size_t i = 0; i < squares.size(); i++ )
//    {
//        const Point* p = &squares[i][0];
//        int n = (int)squares[i].size();
//        polylines(result, &p, &n, 1, true, Scalar(0,255,0), 3, 0);
//    }

    imwrite("/Users/alex/result.jpg",result);
    cv::resize(result,result,Size(ui->result->width(),ui->result->height()));
    cv::cvtColor(result2, result2, CV_BGR2RGB);

    // convert to C format
    IplImage imageIpl = result2;

    // create a QImage container pointing to the image data
    QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);
    //QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_Grayscale8);

    // assign to a QPixmap (may copy)
    QPixmap pix = QPixmap::fromImage(qimg);

    // display image
    //pix = pix.scaled(ui->result->size());
    ui->result->setPixmap(pix);
*/
}
