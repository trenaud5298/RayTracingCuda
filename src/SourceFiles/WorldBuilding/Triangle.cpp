#include "../../HeaderFiles/WorldBuilding/Triangle.h"

Triangle::Triangle(float pos1X, float pos1Y, float pos1Z, float pos2X, float pos2Y, float pos2Z, float pos3X, float pos3Y, float pos3Z, Material* triangleMaterial) {
    this->triangleData = new float[30];
    this->triangleData[0] = 1;
    this->setMaterial(triangleMaterial);
    this->triangleData[6] = pos1X;
    this->triangleData[7] = pos1Y;
    this->triangleData[8] = pos1Z;
    this->triangleData[9] = pos2X;
    this->triangleData[10] = pos2Y;
    this->triangleData[11] = pos2Z;
    this->triangleData[12] = pos3X;
    this->triangleData[13] = pos3Y;
    this->triangleData[14] = pos3Z;
    //GET PLANE DATA:
    this->updateArrayData();

    
}

Triangle::Triangle(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3, Material* triangleMaterial) {
    this->triangleData = new float[30];
    this->triangleData[0] = 1;
    this->setMaterial(triangleMaterial);
    this->triangleData[6] = vertex1->posX;
    this->triangleData[7] = vertex1->posY;
    this->triangleData[8] = vertex1->posZ;
    this->triangleData[9] = vertex2->posX;
    this->triangleData[10] = vertex2->posY;
    this->triangleData[11] = vertex2->posZ;
    this->triangleData[12] = vertex3->posX;
    this->triangleData[13] = vertex3->posY;
    this->triangleData[14] = vertex3->posZ;
    //GET PLANE DATA:
    this->updateArrayData();
}

void Triangle::setMaterial(Material* triangleMaterial) {
    this->triangleData[1] = triangleMaterial->colorR;
    this->triangleData[2] = triangleMaterial->colorG;
    this->triangleData[3] = triangleMaterial->colorB;
    this->triangleData[4] = triangleMaterial->smoothness;
    if(triangleMaterial->lightEmitting == true) {
        this->triangleData[5] = 1.0;
    } else {
        this->triangleData[5] = 0.0;
    }
}

void Triangle::updateArrayData() {
    //PLANAR PART OF DATA
    float vec1X = this->triangleData[9]-this->triangleData[6];
    float vec1Y = this->triangleData[10]-this->triangleData[7]; //POINT 2 - POINT1
    float vec1Z = this->triangleData[11]-this->triangleData[8];
    float vec2X = this->triangleData[12]-this->triangleData[6];
    float vec2Y = this->triangleData[13]-this->triangleData[7]; //POINT 3 - POINT1
    float vec2Z = this->triangleData[14]-this->triangleData[8];

    float normVectorX = vec1Y*vec2Z - vec2Y*vec1Z;
    float normVectorY = vec1Z*vec2X - vec2Z*vec1X;
    float normVectorZ = vec1X*vec2Y - vec2X*vec1Y;
    float inverseNormLength = 1/sqrt(normVectorX*normVectorX + normVectorY*normVectorY + normVectorZ*normVectorZ);
    normVectorX *= inverseNormLength;
    normVectorY *= inverseNormLength;
    normVectorZ *= inverseNormLength;
    float displacementFromOrigin = normVectorX * this->triangleData[9] + normVectorY * this->triangleData[10] + normVectorZ * this->triangleData[11];
    this->triangleData[15] = normVectorX;
    this->triangleData[16] = normVectorY;
    this->triangleData[17] = normVectorZ;
    this->triangleData[18] = displacementFromOrigin;
    //VECTOR PART OF DATA:
        //Vec12
    this->triangleData[19] = this->triangleData[9]-this->triangleData[6];
    this->triangleData[20] = this->triangleData[10]-this->triangleData[7];
    this->triangleData[21] = this->triangleData[11]-this->triangleData[8];
        //Vec13
    this->triangleData[22] = this->triangleData[12]-this->triangleData[6];
    this->triangleData[23] = this->triangleData[13]-this->triangleData[7];
    this->triangleData[24] = this->triangleData[14]-this->triangleData[8];
        //Vec23
    this->triangleData[25] = this->triangleData[12]-this->triangleData[9];
    this->triangleData[26] = this->triangleData[13]-this->triangleData[10];
    this->triangleData[27] = this->triangleData[14]-this->triangleData[11];
    //Compute Double traingle Area:
    this->triangleData[28] = sqrtf(
        powf((this->triangleData[20]*this->triangleData[24] - this->triangleData[21]*this->triangleData[23]),2) +
        powf((this->triangleData[21]*this->triangleData[22] - this->triangleData[19]*this->triangleData[24]),2) +
        powf((this->triangleData[19]*this->triangleData[23] - this->triangleData[20]*this->triangleData[22]),2)
    );

    std::cout<<"Triangle Area: "<<this->triangleData[28]<<std::endl;
}