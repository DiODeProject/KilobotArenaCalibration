#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <ios>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/stitching.hpp>

using namespace cv;

#include <QMainWindow>

// Project includes
#include "calibratearena.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:

    /*!
     * \brief matchConfDoubleConvertor
     * Allows us to convert the slider int into a double for display
     */
    void matchConfDoubleConvertor(int);

    /*!
     * \brief loadImages
     * Method to generate a dialog used for loading the calibration images
     */
    void loadImages();

    /*!
     * \brief capImages
     * Method to capture calibration images from the cameras
     */
    void capImages();


private:
    Ui::MainWindow *ui;

    // private methods
    void testStitching();

    CalibrateArena calibrater;
};

#endif // MAINWINDOW_H
