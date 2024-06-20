#include "../HeaderFiles/Camera.h"
#include <iostream>
#include <cmath>

#define M_PI 3.141592653589793238462643383279502



Camera::Camera() : fieldOfView(90),horizontalRotation(0),verticalRotation(0) {
    this->cameraData[0] = 0.0;
    this->cameraData[1] = 0.0;
    this->cameraData[2] = 0.0;
    this->updateFovScalar();
    this->updateRotationData();
}

Camera::~Camera() {
    std::cout<<"Camera Deconstructing"<<std::endl;
}



void Camera::updateCameraParameters(Settings* settings) {
    int fov = settings->getGraphicsSettingsValue("FieldOfView");
    this->fieldOfView = fov < 150 ? fov : 150;
    this->horizontalCameraSensitivity = settings->getMainSettingsValue("HorizontalCameraSensitivity");
    this->verticalCameraSensitivity = settings->getMainSettingsValue("VerticalCameraSensitivity");
    this->updateFovScalar();
}


void Camera::updateFovScalar() {
    double radianAngle = static_cast<double>(M_PI * this->fieldOfView / 360);
    this->cameraData[3] = static_cast<float>(1.0/std::tan(radianAngle));
}

void Camera::updateRotationData() {
    this->cameraData[4] = std::cos(M_PI*this->horizontalRotation/180);
    this->cameraData[5] = std::cos(M_PI*this->verticalRotation/180);
    this->cameraData[6] = std::sin(M_PI*this->horizontalRotation/180);
    this->cameraData[7] = std::sin(M_PI*this->verticalRotation/180);
}

void Camera::displayInfo() {
    std::cout<<"Position: ( "<<this->cameraData[0]<<", "<<this->cameraData[1]<<", "<<this->cameraData[2]<<" )"<<std::endl;
    std::cout<<"Fov: "<<this->fieldOfView<<std::endl;
    std::cout<<"Fov Scalar: "<<this->cameraData[3]<<std::endl;
}

void Camera::rotateCamera(float horizontalRotation, float verticalRotation) {
    this->horizontalRotation += horizontalRotation;
    float newRotation = this->verticalRotation + verticalRotation;
    this->verticalRotation = (newRotation < -89) ? -89 : (newRotation > 89) ? 89 : newRotation;
    this->updateRotationData();
}


void Camera::moveForward(float distance) {
    /*X-coord*/this->cameraData[0] += distance*this->cameraData[6];
    /*Z-coord*/this->cameraData[2] += distance*this->cameraData[4];
}
void Camera::moveBackward(float distance) {
    /*X-coord*/this->cameraData[0] -= distance*this->cameraData[6];
    /*Z-coord*/this->cameraData[2] -= distance*this->cameraData[4];
}
void Camera::moveRight(float distance) {
    /*X-coord*/this->cameraData[0] += distance*this->cameraData[4];
    /*Z-coord*/this->cameraData[2] -= distance*this->cameraData[6];
}
void Camera::moveLeft(float distance) {
    /*X-coord*/this->cameraData[0] -= distance*this->cameraData[4];
    /*Z-coord*/this->cameraData[2] += distance*this->cameraData[6];
}
void Camera::moveUp(float distance) {
    /*Y-coord*/this->cameraData[1] += distance;
}
void Camera::moveDown(float distance) {
    /*Y-coord*/this->cameraData[1] -= distance;
}

