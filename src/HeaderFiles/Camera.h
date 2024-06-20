#ifndef CAMERA_H
#define CAMERA_H

#include "./Settings.h"

class Camera {

public:
    Camera();
    ~Camera();
    void updateFovScalar();
    void updateCameraParameters(Settings* settings);


    void displayInfo();
    void updateRotationData();
    void rotateCamera(float horizontalRotation, float verticalRotation);
    float cameraData[8];
    float horizontalRotation;
    float verticalRotation;

    //Movement Functions
    void moveForward(float distance);
    void moveBackward(float distance);
    void moveRight(float distance);
    void moveLeft(float distance);
    void moveUp(float distance);
    void moveDown(float distance);


protected:
    int fieldOfView;
    int horizontalCameraSensitivity;
    int verticalCameraSensitivity;
    /*
    Key:
    0:posX
    1:posY
    2:posZ
    3:fovScalar
    4:cosX
    5:cosY
    6:sinX
    7:sinY
    */

};


#endif