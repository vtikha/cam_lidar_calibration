#include <opencv2/core/persistence.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

void detectCharucoBoard(const std::string &imagePath) {
    // Load the image
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        return;
    }

    //resize image to 1014 x 760
    cv::resize(image, image, cv::Size(992, 760));

    //craete opencv window
    cv::namedWindow("Charuco board", cv::WINDOW_NORMAL);
    // resize window
    cv::resizeWindow("Charuco board", 1014, 760);

    // create grayscale image
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat cameramat, distcoeff;

    cameramat = (cv::Mat_<double>(3, 3) << 572.8860985737642,0.0,502.4593683288797,0.0,572.9663599939911,370.12881786909264,0.0,0.0,1.0);
    distcoeff = (cv::Mat_<double>(1, 14) << 7.240687015578669,9.307883595612031,-0.0005999395111158708,0.0005787294266067374,-1.449107846572058,7.304387018547969,11.685436247778302,-0.6762507853900844,0.0,0.0,0.0,0.0,0.0,0.0);

    // Parameters for the Charuco board
    int squaresX = 5; // Number of chessboard squares in X direction
    int squaresY = 7; // Number of chessboard squares in Y direction
    float squareLength = 95.0f; // Square side length (in meters or arbitrary units)
    float markerLength = 65.0f; // Marker side length (in meters or arbitrary units)
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    auto board_width_ = 599;
    auto board_height_ = 845;

    // Create Charuco board
    cv::Ptr<cv::aruco::CharucoBoard> charucoBoard = cv::aruco::CharucoBoard::create(squaresX+1, squaresY+1, squareLength, markerLength, dictionary);
    cv::Mat boardImage;
    charucoBoard->draw( cv::Size(992, 760), boardImage, 10, 1 );
  cv::imwrite("/root/ws_lab0/src/cam_lidar_calibration/data/board.png", boardImage);
    // Detect markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    // Refine and interpolate Charuco corners
    if (markerIds.empty()) 
    {
    return;
    }

    std::vector<cv::Point2f> charucoCorners;
    std::vector<cv::Point2f> charucoCorners_flipped;
    std::vector<int> charucoIds;
    cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, gray, charucoBoard, charucoCorners, charucoIds, cameramat, distcoeff);

    
     cv::Point3d chessboard_bleft_corner((squaresX - 1) * squareLength / 2,
                                      (squaresY - 1) * squareLength / 2, 0);

  std::vector<cv::Point3d> corners_3d;
  for (int y = 0; y < squaresY; y++)
  {
    for (int x = 0; x < squaresX; x++)
    {
      corners_3d.push_back(cv::Point3d(x, y, 0) * squareLength - chessboard_bleft_corner);
    }
  }

// chessboard corners, middle square corners, board corners and centre
  std::vector<cv::Point3d> board_corners_3d;
  // Board corner coordinates from the centre of the chessboard
  board_corners_3d.push_back(cv::Point3d((board_width_ ) / 2.0,
                                         (board_height_ ) / 2.0, 0.0));

  board_corners_3d.push_back(cv::Point3d(-(board_width_ ) / 2.0,
                                         (board_height_ ) / 2.0, 0.0));

  board_corners_3d.push_back(cv::Point3d(-(board_width_ ) / 2.0,
                                         -(board_height_ ) / 2.0, 0.0));

  board_corners_3d.push_back(cv::Point3d((board_width_ ) / 2.0,
                                         -(board_height_) / 2.0, 0.0));

  // Board centre coordinates from the centre of the chessboard (due to
  // incorrect placement of chessboard on board)
  board_corners_3d.push_back(
      cv::Point3d(0.0, 0.0, 0.0));


  std::vector<cv::Point2d> inner_cbcorner_pixels, board_image_pixels;
  cv::Mat rvec(3, 3, cv::DataType<double>::type);  // Initialization for pinhole
                                                   // and fisheye cameras
  cv::Mat tvec(3, 1, cv::DataType<double>::type);



   // Pinhole model
      auto valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoBoard, cameramat, distcoeff, rvec, tvec);
      // translate the origin to the center of the chessboard
      Eigen::Vector3d chessboard_center_corner((squaresX + 1) * squareLength / 2,
                                               (squaresY + 1) * squareLength / 2, 0);

      // Modify rvec, tvec to match the position and orientation of the chessboard frame
      cv::Mat rot_mat;
      cv::Rodrigues(rvec, rot_mat);
      Eigen::Matrix3d rot_mat_eigen;
      cv::cv2eigen(rot_mat, rot_mat_eigen);
      Eigen::Vector3d tvec_eigen;
      cv::cv2eigen(tvec, tvec_eigen);     
      tvec_eigen = tvec_eigen + rot_mat_eigen * chessboard_center_corner;
      cv::eigen2cv(tvec_eigen, tvec);
      // Flip the z and x axes
      rot_mat_eigen.col(2) = -rot_mat_eigen.col(2);
      rot_mat_eigen.col(0) = -rot_mat_eigen.col(0);
      cv::eigen2cv(rot_mat_eigen, rot_mat);
      cv::Rodrigues(rot_mat, rvec);
      cv::projectPoints(corners_3d, rvec, tvec, cameramat, distcoeff, inner_cbcorner_pixels);
      cv::projectPoints(board_corners_3d, rvec, tvec, cameramat, distcoeff, board_image_pixels);

    for (std::size_t i = 0; i < board_image_pixels.size(); i++)
  {
    if (i == 4)
    {
      cv::circle(image, board_image_pixels[i], 6, CV_RGB(255, 0, 0), -1);
    }
    else if (i == 3)
    {
      cv::circle(image, board_image_pixels[i], 6, CV_RGB(0, 255, 0), -1);
    }
    else if (i == 2)
    {
      cv::circle(image, board_image_pixels[i], 6, CV_RGB(0, 0, 255), -1);
    }
    else if (i == 1)
    {
      cv::circle(image, board_image_pixels[i], 6, CV_RGB(255, 255, 0), -1);
    }
    else if (i == 0)
    {
      cv::circle(image, board_image_pixels[i], 6, CV_RGB(0, 255, 255), -1);
    }
  }

  for (auto& point : inner_cbcorner_pixels)
  {
    cv::circle(image, point, 4, CV_RGB(0, 0, 255), -1);
  }

    if(valid) cv::aruco::drawAxis(image, cameramat, distcoeff, rot_mat, tvec, 100);
    // std::cout << "rvec: " << rvec << std::endl;
    // std::cout << "tvec: " << tvec << std::endl;
    // cv::aruco::drawDetectedCornersCharuco(image, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
    cv::imshow("Charuco board", image);
    cv::waitKey(0);

    cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  cv::Mat z = cv::Mat(cv::Point3d(0., 0., -1.));  // TODO: why is this normal -1 in z? Surabhi's is just 1
  cv::Mat  chessboard_normal = rmat * z;

  std::vector<cv::Point3d> corner_vectors;
  for (auto& corner : board_corners_3d)
  {
    cv::Mat m(rmat * cv::Mat(corner).reshape(1) + tvec);
    corner_vectors.push_back(cv::Point3d(m));
  }
  // corners_vector, chessboard_normal
    std::cout << corner_vectors[4].x << ", " << corner_vectors[4].y << ", " <<  corner_vectors[4].z  << std::endl;
    std::cout << chessboard_normal.at<double>(0,0) << ", " << chessboard_normal.at<double>(0,1) << ", " << chessboard_normal.at<double>(0,2)<< std::endl;
    std::cout << corner_vectors[0].x << ", " << corner_vectors[0].y << ", " <<  corner_vectors[0].z  << std::endl;
    std::cout << corner_vectors[1].x << ", " << corner_vectors[1].y << ", " <<  corner_vectors[1].z  << std::endl;
    std::cout << corner_vectors[2].x << ", " << corner_vectors[2].y << ", " <<  corner_vectors[2].z  << std::endl;
    std::cout << corner_vectors[3].x << ", " << corner_vectors[3].y << ", " <<  corner_vectors[3].z  << std::endl;


  double pixdiagonal = sqrt(pow(inner_cbcorner_pixels.front().x - inner_cbcorner_pixels.back().x, 2) +
                            (pow(inner_cbcorner_pixels.front().y - inner_cbcorner_pixels.back().y, 2)));

  double len_diagonal =
      sqrt(pow(corners_3d.front().x - corners_3d.back().x, 2) + (pow(corners_3d.front().y - corners_3d.back().y, 2)));

  auto metreperpixel_cbdiag_ = len_diagonal / (1000 * pixdiagonal);

  std::cout << "metreperpixel_cbdiag_: " << metreperpixel_cbdiag_ << std::endl;
   
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return 1;
    }

    detectCharucoBoard(argv[1]);
    return 0;
}