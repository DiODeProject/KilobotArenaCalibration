#ifndef CALIBRATEARENA_H
#define CALIBRATEARENA_H
#include <ios>
#include <vector>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

// allow easy addressing of OpenCV functions
using namespace cv;
using namespace std;

// Qt base include
#include <QObject>
#include <QPoint>
#include <QPixmap>
#include <QPushButton>

class stitchThread;

/*!
 * \brief The CalibrateArena class
 *
 * This is the class responsible for using OpenCV's stitch pipeline to align the cameras covering the arena, and
 * generate the transforms required for both alignment and combination into a single image. The stitcher pipeline
 * creates a final image with a perspective transformation required to move to a squared image, so corner markers
 * are used in the final stage to square the calibration and set the arena dimensions.
 *
 * A single final image is generated as this removes the need for removing duplicate Kilobots while tracking, however
 * the individual transformed images for each camera are retained in order to allow the LEDs of Kilobots in the overlap
 * to be viewed from multiple directions, reducing dropouts.
 */
class CalibrateArena : public QObject
{
    Q_OBJECT
public:
    explicit CalibrateArena(QPoint smallImageSize = QPoint(300,300), QObject *parent = 0);
    ~CalibrateArena();

signals:
    /*!
     * \brief errorMessage
     * Qt signal to update the UI message QLabel
     */
    void errorMessage(QString);

    void setImage0(QPixmap);
    void setImage1(QPixmap);
    void setImage2(QPixmap);
    void setImage3(QPixmap);

    void setFeaturesImage0(QPixmap);
    void setFeaturesImage1(QPixmap);
    void setFeaturesImage2(QPixmap);
    void setFeaturesImage3(QPixmap);

    void setStitchedImage(QPixmap);

    void setSquaredImage(QPixmap);

public slots:

    /*!
     * \brief extractFeatures
     * Extract relevant features from the camera calibration images.
     */
    void extractFeatures();

    /*!
     * \brief setCalibrationImages
     * Set the calibration images
     */
    void setCalibrationImages(vector <Mat>);

    /*!
     * \brief setFeatureFinderThreshold
     * Accessor slot
     */
    void setFeatureFinderThreshold(int);

    /*!
     * \brief setMatcherThreshold
     * Accessor slot
     */
    void setMatcherThreshold(int);

    /*!
     * \brief stitchImages
     * Use the existing feature matches to stitch the images
     */
    void stitchImages();

    /*!
     * \brief stitcherFinished
     * Called by the stitcher thread when it completes
     */
    void stitcherFinished();

    /*!
     * \brief pointSelected
     * Select the points used to square the arena
     */
    void pointSelected(QPoint);

    /*!
     * \brief resetPoint
     * clear last point
     */
    void resetPoint();

    /*!
     * \brief squareArena
     * Square the arena using the selected points
     */
    void squareArena();

    /*!
     * \brief saveCalibration
     * Save the calibration matrices to an OpenCV FileStorage format
     */
    void saveCalibration();

    /*!
     * \brief zoomMove
     * Slot to facilitate zooming and panning the final image
     */
    void zoomMove(QPoint);

    /*!
     * \brief zoomMoveDone
     * Slot to facilitate finishing zooming and panning the final image
     */
    void zoomMoveDone();


private:
    // private members
    /*!
     * \brief cameraCalibrationImages
     * A vector containing the calibration images from the cameras.
     */
    vector <Mat> cameraCalibrationImages;
    /*!
     * \brief smallImageSize
     * Assigned in the constructor
     */
    QPoint smallImageSize;
    /*!
     * \brief featureFinderThreshold
     * The threshold value for the Surf feature finder
     */
    int featureFinderThreshold = 10; // default

    /*!
     * \brief matcherThreshold
     * The threshold for the pairwise matcher
     */
    float matcherThreshold = 0.6f; // default

    /*!
     * \brief features
     * This data must be passed from the feature extraction to the Homography estimator/refiner
     */
    vector<detail::ImageFeatures> features;
    /*!
     * \brief pairwise_matches
     * This data must be passed from the feature extraction to the Homography estimator/refiner
     */
    vector<detail::MatchesInfo> pairwise_matches;
    /*!
     * \brief goodMatches
     * Flag that we have successful matching of the images, defaults to false
     */
    bool goodMatches = false;

    /*!
     * \brief arenaCorners
     * A vector containing the corners of the arena, used when squaring the stitched image.
     * Currently these are user selected, however future work may automate corner selection.
     */
    QVector < QPoint > arenaCorners;

    /*!
     * \brief fullSizeFinalIm
     * Full size version of the final image. This is used when zooming and panning in the UI
     */
    Mat fullSizeFinalIm;

    // private methods
    /*!
     * \brief thread
     * Stitcher thread allowing the sandboxing of the stitching process as it can sometimes hang
     */
    stitchThread * thread = NULL;

    QPushButton * stitchButton;
};



#endif // CALIBRATEARENA_H
