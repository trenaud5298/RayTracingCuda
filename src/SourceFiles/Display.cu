#include "../HeaderFiles/Display.cuh"
#include <iostream>
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <chrono>
#include <thread>
#include "../../lib/SDL/include/SDL.h"
#include "../../lib/SDL/include/SDL_ttf.h"

#include "../HeaderFiles/Camera.h"
#include "../HeaderFiles/Button.h"


#define MAIN_MENU 0
#define RUNNING 1
#define PAUSED 2


__device__ float3 getVector(int x,int y,int imageWidth,int imageHeight,float cosX, float cosY, float sinX, float sinY,float fovScalar) {
    float heightToWidthRatio = static_cast<float>(imageHeight)/imageWidth;
    float3 rayVector = make_float3((static_cast<float>(2*x-imageWidth)/imageWidth),heightToWidthRatio*(static_cast<float>(imageHeight-2*y)/imageHeight),static_cast<float>(1.0*fovScalar));
    float inverseLength = rsqrtf(rayVector.x*rayVector.x+rayVector.y*rayVector.y+rayVector.z*rayVector.z);
    rayVector.x *= inverseLength;
    rayVector.y *= inverseLength;
    rayVector.z *= inverseLength;

    float verticalZ = rayVector.y*sinY + rayVector.z*cosY;
    return make_float3(rayVector.x*cosX+verticalZ*sinX,rayVector.y*cosY-rayVector.z*sinY,verticalZ*cosX-rayVector.x*sinX);
}


__device__ float3 getSkyBoxColor(float3 rayVector, float angleOfSun) {
    /* FUTURE IDEA:
     * Use an Image(s) of a SkyBox and In Python
     * Convert all of the RGB Values for each
     * pixels to floats between 0 and 1 and then
     * store that data in a file to be read in
     * for this program and then store that data in
     * an array and be prepared to convert from
     * a vector to a pixel from the skybox
    */
    float3 skyColor;
    float3 sunColor = make_float3(1.0f,1.0f,1.0f);
    if(angleOfSun > 0) {
        float angleBetweenRayAndSun = acos(rayVector.x*__cosf(angleOfSun) + rayVector.y*__sinf(angleOfSun));
        if(angleBetweenRayAndSun < 0.05) {
            skyColor.x = sunColor.x;
            skyColor.y = sunColor.y;
            skyColor.z = sunColor.z;
            return skyColor;
        }
        float t = 0.5f*(1.0f+(__sinf(acos(rsqrtf(rayVector.x*rayVector.x+rayVector.z*rayVector.z)*(rayVector.x*rayVector.x+rayVector.z*rayVector.z)))));
        float3 topSkyColor = make_float3(64.0f/255.0f,156.0f/255.0f,255.0f/255.0f);
        float3 bottomSkyColor = make_float3(1.0f,1.0f,1.0f);
        skyColor.x = topSkyColor.x*t + bottomSkyColor.x*(1-t);
        skyColor.y = topSkyColor.y*t + bottomSkyColor.y*(1-t);
        skyColor.z = topSkyColor.z*t + bottomSkyColor.z*(1-t);
        if(angleBetweenRayAndSun < 0.075) {
            int m = 1-10*fabs(angleBetweenRayAndSun-0.05);
            float colorTintX = min(1.0f,skyColor.x*m + sunColor.x*(1-m));
            float colorTintY = min(1.0f,skyColor.y*m + sunColor.y*(1-m));
            float colorTintZ = min(1.0f,skyColor.z*m + sunColor.z*(1-m));
            skyColor.x *= colorTintX;
            skyColor.y *= colorTintY;
            skyColor.z *= colorTintZ;

        }
        return skyColor;

    } else {
        skyColor.x = 0;
        skyColor.y = 0;
        skyColor.z = 0;
        return skyColor;
    }

}

__device__ float computeCrossProductLength(float3 vec1, float3 vec2) {
    return sqrtf(
        powf((vec1.y*vec2.z-vec2.y*vec1.z),2)+
        powf((vec1.z*vec2.x-vec2.z*vec1.x),2)+
        powf((vec1.x*vec2.y-vec2.x*vec1.y),2)
    );
}

__device__ float3 getRandomBounceVector(float3 normalVector, curandState* randomGenerator) {
    float3 randomBounceVector = make_float3(1.0f,0,0);
    float x;
    float y;
    float z;
    float squaredLength;
    for(int i = 0; i < 100; i ++) {
        x = curand_normal(randomGenerator);
        y = curand_normal(randomGenerator);
        z = curand_normal(randomGenerator);
        squaredLength = sqrtf(x*x+y*y+z*z); 
        if(squaredLength < 1) {
            randomBounceVector.x = x/squaredLength;
            randomBounceVector.y = y/squaredLength;
            randomBounceVector.z = z/squaredLength;
            break;
        }
    }
    if((normalVector.x*randomBounceVector.x + normalVector.y*randomBounceVector.y + normalVector.z*randomBounceVector.z) < 0) {
        randomBounceVector.x *= -1;
        randomBounceVector.y *= -1;
        randomBounceVector.z *= -1;

    }

    return randomBounceVector;
}

__device__ float* handleRayIntersection(float3 rayOrigin, float3 rayVector, float**worldObjectData, int numWorldObjects, bool* intersected, float* shortestIntersectionDistance) {
    float* intersectionObject;
    for(int i = 0; i < numWorldObjects; i++) {
        switch(static_cast<int>(worldObjectData[i][0])) {
            case 0:
                float sphereVecX = worldObjectData[i][6] - rayOrigin.x;
                float sphereVecY = worldObjectData[i][7] - rayOrigin.y;
                float sphereVecZ = worldObjectData[i][8] - rayOrigin.z;
                float sphereRadius = worldObjectData[i][9];
                float dotProduct = (sphereVecX*rayVector.x + sphereVecY*rayVector.y + sphereVecZ*rayVector.z); //LENGTH t1c
                if(dotProduct<0){break;}
                float distanceToSphereCenter = sqrtf(sphereVecX*sphereVecX + sphereVecY*sphereVecY + sphereVecZ*sphereVecZ);
                float rayPerpendicularDistanceToCenter = sqrtf(distanceToSphereCenter*distanceToSphereCenter - dotProduct*dotProduct);
                if(rayPerpendicularDistanceToCenter > sphereRadius) {break;}
                float distanceToSphereEdge = sqrtf(sphereRadius*sphereRadius - rayPerpendicularDistanceToCenter*rayPerpendicularDistanceToCenter);
                float intersectionDistance1 = dotProduct - distanceToSphereEdge;
                float intersectionDistance2 = dotProduct + distanceToSphereEdge;
                if(intersectionDistance1 > 0 ){
                    if(!*intersected) {
                        *intersected = true;
                        *shortestIntersectionDistance = intersectionDistance1;
                        intersectionObject = worldObjectData[i];
                    } else if(intersectionDistance1 < *shortestIntersectionDistance){
                        *intersected = true;
                        *shortestIntersectionDistance = intersectionDistance1;
                        intersectionObject = worldObjectData[i];
                    }
                }
                // if(!*intersected) {
                //     if(intersectionDistance1 > 0) {
                //         *shortestIntersectionDistance = intersectionDistance1;
                //         intersectionObject = worldObjectData[i];
                //         *intersected = true;
                //     }
                //     if(intersectionDistance2 > 0 && intersectionDistance2 < intersectionDistance1) {
                //         *shortestIntersectionDistance = intersectionDistance2;
                //         intersectionObject = worldObjectData[i];
                //         *intersected = true;
                //     }
                // } else{
                //     if(intersectionDistance1 > 0 && intersectionDistance1 < *shortestIntersectionDistance) {
                //         *shortestIntersectionDistance = intersectionDistance1;
                //         intersectionObject = worldObjectData[i];
                //     }
                //     if(intersectionDistance2 > 0 && intersectionDistance2 < *shortestIntersectionDistance) {
                //         *shortestIntersectionDistance = intersectionDistance2;
                //         intersectionObject = worldObjectData[i];
                //     }
                // }
                break;
            
            case 1:
                float distanceToIntersection = 
                (worldObjectData[i][18]-worldObjectData[i][15]*rayOrigin.x-worldObjectData[i][16]*rayOrigin.y-worldObjectData[i][17]*rayOrigin.z)
                /(worldObjectData[i][15]*rayVector.x + worldObjectData[i][16]*rayVector.y + worldObjectData[i][17]*rayVector.z);
                if(distanceToIntersection < 0 || isnan(distanceToIntersection)) {break;}
                //Determine if point is in triangle
                float3 intersectionPoint = make_float3(
                    rayOrigin.x+rayVector.x*distanceToIntersection, 
                    rayOrigin.y+rayVector.y*distanceToIntersection, 
                    rayOrigin.z+rayVector.z*distanceToIntersection
                );

                float3 vecA = make_float3(
                    intersectionPoint.x-worldObjectData[i][6],
                    intersectionPoint.y-worldObjectData[i][7],
                    intersectionPoint.z-worldObjectData[i][8]
                );    
                float triangle1Area=computeCrossProductLength(vecA,make_float3(worldObjectData[i][19],worldObjectData[i][20],worldObjectData[i][21]));
                float triangle2Area=computeCrossProductLength(vecA,make_float3(worldObjectData[i][22],worldObjectData[i][23],worldObjectData[i][24]));
                float triangle3Area=computeCrossProductLength(make_float3(intersectionPoint.x-worldObjectData[i][9],intersectionPoint.y-worldObjectData[i][10],intersectionPoint.z-worldObjectData[i][11]),make_float3(worldObjectData[i][25],worldObjectData[i][26],worldObjectData[i][27]));
                if(fabs(triangle1Area+triangle2Area+triangle3Area-worldObjectData[i][28]) > 0.0001) {
                    break;
                }
                if(!*intersected && distanceToIntersection>0) {
                    *shortestIntersectionDistance = distanceToIntersection;
                    intersectionObject = worldObjectData[i];
                    *intersected = true;
                } else if(distanceToIntersection>0 && distanceToIntersection < *shortestIntersectionDistance) {
                    *shortestIntersectionDistance = distanceToIntersection;
                    intersectionObject = worldObjectData[i];
                }
                break;
        }
    }
    return intersectionObject;
}


//Alternate Method Of Handling Intersection by Avoiding the Use of Bool Value (Was useful for troubleshooting!)
__device__ float* handleRayIntersection2(float3 rayOrigin, float3 rayVector, float**worldObjectData, int numWorldObjects, float* shortestIntersectionDistance,float* troubleShootData,int x, int y) {
    float* intersectionObject;
    for(int i = 0; i < numWorldObjects; i++) {
        switch(static_cast<int>(worldObjectData[i][0])) {
            case 0:
                float sphereVecX = worldObjectData[i][6] - rayOrigin.x;
                float sphereVecY = worldObjectData[i][7] - rayOrigin.y;
                float sphereVecZ = worldObjectData[i][8] - rayOrigin.z;
                float sphereRadius = worldObjectData[i][9];
                float dotProduct = (sphereVecX*rayVector.x + sphereVecY*rayVector.y + sphereVecZ*rayVector.z); //LENGTH t1c
                if(dotProduct<0){break;}
                float distanceToSphereCenter = sqrtf(sphereVecX*sphereVecX + sphereVecY*sphereVecY + sphereVecZ*sphereVecZ);
                float rayPerpendicularDistanceToCenter = sqrtf(distanceToSphereCenter*distanceToSphereCenter - dotProduct*dotProduct);
                if(rayPerpendicularDistanceToCenter > sphereRadius) {break;}
                float distanceToSphereEdge = sqrtf(sphereRadius*sphereRadius - rayPerpendicularDistanceToCenter*rayPerpendicularDistanceToCenter);
                float intersectionDistance1 = dotProduct - distanceToSphereEdge;
                float intersectionDistance2 = dotProduct + distanceToSphereEdge;
                if(intersectionDistance1>0 && intersectionDistance1 < *shortestIntersectionDistance) {
                    *shortestIntersectionDistance = intersectionDistance1;
                    intersectionObject = worldObjectData[i];
                }
                if(intersectionDistance2>0 && intersectionDistance2 < *shortestIntersectionDistance) {
                    *shortestIntersectionDistance = intersectionDistance2;
                    intersectionObject = worldObjectData[i];
                }
                break;
            
            case 1:
                float distanceToIntersection = 
                (worldObjectData[i][18]-worldObjectData[i][15]*rayOrigin.x-worldObjectData[i][16]*rayOrigin.y-worldObjectData[i][17]*rayOrigin.z)
                /(worldObjectData[i][15]*rayVector.x + worldObjectData[i][16]*rayVector.y + worldObjectData[i][17]*rayVector.z);
                if(distanceToIntersection < 0 || isnan(distanceToIntersection)) {break;}
                //Determine if point is in triangle
                float3 intersectionPoint = make_float3(
                    rayOrigin.x+rayVector.x*distanceToIntersection, 
                    rayOrigin.y+rayVector.y*distanceToIntersection, 
                    rayOrigin.z+rayVector.z*distanceToIntersection
                );

                float3 vecA = make_float3(
                    intersectionPoint.x-worldObjectData[i][6],
                    intersectionPoint.y-worldObjectData[i][7],
                    intersectionPoint.z-worldObjectData[i][8]
                );    
                float triangle1Area=computeCrossProductLength(vecA,make_float3(worldObjectData[i][19],worldObjectData[i][20],worldObjectData[i][21]));
                float triangle2Area=computeCrossProductLength(vecA,make_float3(worldObjectData[i][22],worldObjectData[i][23],worldObjectData[i][24]));
                float triangle3Area=computeCrossProductLength(make_float3(intersectionPoint.x-worldObjectData[i][9],intersectionPoint.y-worldObjectData[i][10],intersectionPoint.z-worldObjectData[i][11]),make_float3(worldObjectData[i][25],worldObjectData[i][26],worldObjectData[i][27]));
                if(fabs(triangle1Area+triangle2Area+triangle3Area-worldObjectData[i][28]) > 0.0001) {
                    break;
                }
                if(distanceToIntersection>0 && distanceToIntersection<*shortestIntersectionDistance) {
                    *shortestIntersectionDistance = distanceToIntersection;
                    intersectionObject = worldObjectData[i];
                }
                break;
        }
    }
    if(!intersectionObject) {
        intersectionObject = new float[30];
        intersectionObject[0] = -1;
    }
    if(x==600 && y==400) {
        troubleShootData[0] = *shortestIntersectionDistance;
        troubleShootData[1] = intersectionObject[1];
        troubleShootData[2] = intersectionObject[2];
        troubleShootData[3] = intersectionObject[3];
        troubleShootData[4] = intersectionObject[5];
        troubleShootData[5] = intersectionObject[6];
        troubleShootData[6] = intersectionObject[7];
        troubleShootData[7] = intersectionObject[8];
    }
    return intersectionObject;
}

__device__ float3 getNormalVector(float* intersectionObject, float3 intersectionPoint, float3 rayVector) {
    float3 normalVector;
    switch(static_cast<int>(intersectionObject[0])) {
        case 0:
            normalVector.x = intersectionPoint.x-intersectionObject[6];
            normalVector.y = intersectionPoint.y-intersectionObject[7];
            normalVector.z = intersectionPoint.z-intersectionObject[8];
            break;

        case 1:
            normalVector.x = intersectionObject[15];
            normalVector.y = intersectionObject[16];
            normalVector.z = intersectionObject[17];
            break;
    }
    if((normalVector.x*rayVector.x + normalVector.y*rayVector.y + normalVector.z*rayVector.z)>0) {
        normalVector.x*=-1;
        normalVector.y*=-1;
        normalVector.z*=-1;
    }
    //NORMALIZE VECTOR:
    float inverseLength = rsqrtf(normalVector.x*normalVector.x + normalVector.y*normalVector.y + normalVector.z*normalVector.z);
    normalVector.x *= inverseLength;
    normalVector.y *= inverseLength;
    normalVector.z *= inverseLength;
    return normalVector;

}

__global__ void RayTracingKernelSimple(unsigned int seed, int imageWidth, int imageHeight,int numWorldObjects, float* cameraData, unsigned char* imageData, float** worldObjectData,float* troubleShootData) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < imageWidth && y < imageHeight) {
        int pixelIndex = (y * imageWidth + x) * 3;
        
        curandState randomGenerator;
        curand_init(seed+x+y*imageWidth,0,0,&randomGenerator);
        //Rotate Vector:
        float3 rayVector = getVector(x,y,imageWidth,imageHeight,cameraData[4],cameraData[5],cameraData[6],cameraData[7],cameraData[3]);
        float3 rayOrigin = make_float3(cameraData[0],cameraData[1],cameraData[2]);
        float3 color = make_float3(1,1,1);
        float3 intersectionPoint;
        float3 bounceVec;
        float3 normVector;
        float shortestIntersectionDistance;

        shortestIntersectionDistance = 999999;
        float* intersectionObject = handleRayIntersection2(rayOrigin,rayVector,worldObjectData,numWorldObjects,&shortestIntersectionDistance,troubleShootData,x,y);
        
        if(shortestIntersectionDistance < 9999){
            color.x*=intersectionObject[1];
            color.y*=intersectionObject[2];
            color.z*=intersectionObject[3];
            intersectionPoint = make_float3(
            rayOrigin.x + rayVector.x*shortestIntersectionDistance,
            rayOrigin.y + rayVector.y*shortestIntersectionDistance,
            rayOrigin.z + rayVector.z*shortestIntersectionDistance
            );
            normVector = getNormalVector(intersectionObject,intersectionPoint,rayVector);
            bounceVec = getRandomBounceVector(normVector,&randomGenerator);
            if(x==600&&y==400){
                troubleShootData[0] = normVector.x;
                troubleShootData[1] = normVector.y;
                troubleShootData[2] = normVector.z;
                troubleShootData[3] = bounceVec.x;
                troubleShootData[4] = bounceVec.y;
                troubleShootData[5] = bounceVec.z;
            }
        } else{
            float3 skyColor = getSkyBoxColor(rayVector,0.75);
            color.x *= skyColor.x;
            color.y *= skyColor.y;
            color.z *= skyColor.z;
            imageData[pixelIndex] = static_cast<unsigned char>(255.0f * color.x); // Red
            imageData[pixelIndex + 1] = static_cast<unsigned char>(255.0f * color.y); // Green
            imageData[pixelIndex + 2] = static_cast<unsigned char>(255.0f * color.z); // Blue
            return;
        }
        imageData[pixelIndex] = static_cast<unsigned char>(255.0f * color.x); // Red
        imageData[pixelIndex + 1] = static_cast<unsigned char>(255.0f * color.y); // Green
        imageData[pixelIndex + 2] = static_cast<unsigned char>(255.0f * color.z); 
        
    }
};





__global__ void RayTracingKernel(unsigned int seed, int frame, int imageWidth, int imageHeight,int numWorldObjects, float* cameraData, unsigned char* imageData, float** worldObjectData,float* troubleShootData) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < imageWidth && y < imageHeight) {
        int pixelIndex = (y * imageWidth + x) * 3;
        
        curandState randomGenerator;
        curand_init(seed+x+y*imageWidth+frame*79231,0,0,&randomGenerator);
        //Rotate Vector:
        float3 rayVector = getVector(x,y,imageWidth,imageHeight,cameraData[4],cameraData[5],cameraData[6],cameraData[7],cameraData[3]);
        float3 rayOrigin = make_float3(cameraData[0],cameraData[1],cameraData[2]);
        float3 color = make_float3(1,1,1);
        float3 lightColor = make_float3(0.0f,0.0f,0.0f);
        float3 normalVector;
        bool intersected;
        float shortestIntersectionDistance;
        float* intersectionObject;
        for(int i =0; i < 10; i ++) {
            intersected = false;
            shortestIntersectionDistance = 999999999999;
            intersectionObject = handleRayIntersection(rayOrigin,rayVector,worldObjectData,numWorldObjects,&intersected,&shortestIntersectionDistance);
            if(intersected){
                if(intersectionObject[5] > 0.5) {
                    lightColor.x = intersectionObject[1];
                    lightColor.y = intersectionObject[2];
                    lightColor.z = intersectionObject[3];
                    break;
                } else {
                    color.x*=intersectionObject[1];
                    color.y*=intersectionObject[2];
                    color.z*=intersectionObject[3];
                    rayOrigin.x += rayVector.x*shortestIntersectionDistance;
                    rayOrigin.y += rayVector.y*shortestIntersectionDistance;
                    rayOrigin.z += rayVector.z*shortestIntersectionDistance;
                    normalVector = getNormalVector(intersectionObject,rayOrigin,rayVector);
                    rayOrigin.x+=normalVector.x*0.000001;
                    rayOrigin.y+=normalVector.y*0.000001;
                    rayOrigin.z+=normalVector.z*0.000001;
                    rayVector = getRandomBounceVector(normalVector,&randomGenerator);
                }
            }
        }
        color.x *= lightColor.x;
        color.y *= lightColor.y;
        color.z *= lightColor.z;
        
        float proportionalConstant = 1.0/static_cast<float>(frame);
        if(x==600 && y==400) {
            troubleShootData[0] = lightColor.x;
            troubleShootData[1] = lightColor.x;
            troubleShootData[2] = lightColor.x;
            troubleShootData[3] = intersectionObject[5];
            troubleShootData[4] = color.x;
            troubleShootData[5] = color.x;
            troubleShootData[6] = color.x;
            troubleShootData[7] = proportionalConstant;
            troubleShootData[8] = frame;

        }
        imageData[pixelIndex] = imageData[pixelIndex] * (1-proportionalConstant) + proportionalConstant*static_cast<unsigned char>(255.0f * color.x); // Red
        imageData[pixelIndex + 1] = imageData[pixelIndex+1] * (1-proportionalConstant) + proportionalConstant*static_cast<unsigned char>(255.0f * color.y); // Green
        imageData[pixelIndex + 2] = imageData[pixelIndex+2] * (1-proportionalConstant) + proportionalConstant*static_cast<unsigned char>(255.0f * color.z);
    }
};

__global__ void DiffuseRender(unsigned int seed, int frame,int maxRayBounces, int imageWidth, int imageHeight,int numWorldObjects, float* cameraData, float* imageDataArray, float** worldObjectData,float* troubleShootData) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < imageWidth && y < imageHeight) {
        int pixelIndex = (y * imageWidth + x) * 3;
        
        curandState randomGenerator;
        curand_init(seed+frame*79231,x+y*imageWidth,0,&randomGenerator);
        //Rotate Vector:
        float3 rayVector = getVector(x,y,imageWidth,imageHeight,cameraData[4],cameraData[5],cameraData[6],cameraData[7],cameraData[3]);
        float3 rayOrigin = make_float3(cameraData[0],cameraData[1],cameraData[2]);
        float3 color = make_float3(1,1,1);
        float3 lightColor = make_float3(0.1f,0.1f,0.1f);
        float3 normalVector;
        bool intersected;
        float shortestIntersectionDistance;
        float* intersectionObject;
        for(int i =0; i < maxRayBounces; i ++) {
            intersected = false;
            shortestIntersectionDistance = 999999999999;
            intersectionObject = handleRayIntersection(rayOrigin,rayVector,worldObjectData,numWorldObjects,&intersected,&shortestIntersectionDistance);
            if(intersected){
                if(intersectionObject[5] > 0.5) {
                    lightColor.x = intersectionObject[1];
                    lightColor.y = intersectionObject[2];
                    lightColor.z = intersectionObject[3];
                    break;
                } else {
                    color.x*=intersectionObject[1];
                    color.y*=intersectionObject[2];
                    color.z*=intersectionObject[3];
                    rayOrigin.x += rayVector.x*shortestIntersectionDistance;
                    rayOrigin.y += rayVector.y*shortestIntersectionDistance;
                    rayOrigin.z += rayVector.z*shortestIntersectionDistance;
                    normalVector = getNormalVector(intersectionObject,rayOrigin,rayVector);
                    rayOrigin.x+=normalVector.x*0.000001;
                    rayOrigin.y+=normalVector.y*0.000001;
                    rayOrigin.z+=normalVector.z*0.000001;
                    rayVector = getRandomBounceVector(normalVector,&randomGenerator);
                }
            }
        }
        color.x *= lightColor.x;
        color.y *= lightColor.y;
        color.z *= lightColor.z;
        
        imageDataArray[pixelIndex] = imageDataArray[pixelIndex] + color.x; // Red
        imageDataArray[pixelIndex + 1] = imageDataArray[pixelIndex+1] + color.y; // Green
        imageDataArray[pixelIndex + 2] = imageDataArray[pixelIndex+2] + color.z;

        if(x==600 && y==400) {
            troubleShootData[0] = imageDataArray[pixelIndex];
            troubleShootData[1] = imageDataArray[pixelIndex+1];
            troubleShootData[2] = imageDataArray[pixelIndex+2];
            troubleShootData[3] = color.x;
            troubleShootData[4] = color.y;
            troubleShootData[5] = color.z;
        }
    }
};


__global__ void updateImage(int frame, int imageWidth, int imageHeight,float* imageDataArray,unsigned char* imageData,float* troubleShootData) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < imageWidth && y < imageHeight) {
        int pixelIndex = (y * imageWidth + x) * 3;
        
        
        imageData[pixelIndex] = static_cast<unsigned char>(255*fminf(1,static_cast<float>(imageDataArray[pixelIndex]/(0.2*static_cast<float>(frame))))); // Red
        imageData[pixelIndex + 1] = static_cast<unsigned char>(255*fminf(1,static_cast<float>(imageDataArray[pixelIndex+1]/(0.2*static_cast<float>(frame))))); // Green
        imageData[pixelIndex + 2] = static_cast<unsigned char>(255*fminf(1,static_cast<float>(imageDataArray[pixelIndex+2]/(0.2*static_cast<float>(frame)))));
        if(x==600 && y==400) {
            troubleShootData[0] = imageData[pixelIndex];
            troubleShootData[1] = imageData[pixelIndex+1];
            troubleShootData[2] = imageData[pixelIndex+2];
            troubleShootData[3] = imageDataArray[pixelIndex];
            troubleShootData[4] = imageDataArray[pixelIndex+1];
            troubleShootData[5] = imageDataArray[pixelIndex+2];
        }
    }
};





Display::Display(int displayWidth,int displayHeight,Camera* viewCamera): displayWidth(displayWidth),displayHeight(displayHeight),viewCamera(viewCamera),displayState(MAIN_MENU) {
    this->imageData = new unsigned char[this->displayWidth * this->displayHeight * 3];
    std::cout<<"Display Created"<<std::endl;

    //Initializes SDL VIDEO
    if (SDL_Init(SDL_INIT_VIDEO) != 0 || TTF_Init() == -1) {
        std::cout<<"MAJOR ERROR: FAILURE TO LOAD SDL"<<std::endl;
    }

    //Make Display Window
    this->displayWindow = SDL_CreateWindow("RayTracing", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, this->displayWidth, this->displayHeight, SDL_WINDOW_RESIZABLE);
    if (this->displayWindow==NULL) {
        std::cout<<"MAJOR ERROR: FAILURE TO CREATE SDL WINDOW"<<std::endl;
    }
    this->resetCursor();
    
    //Initialize and Create SDL Renderer to display image
    this->imageRenderer = SDL_CreateRenderer(this->displayWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_SetRenderDrawBlendMode(this->imageRenderer, SDL_BLENDMODE_BLEND);
    
    this->imageTexture = SDL_CreateTexture(this->imageRenderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, this->displayWidth, this->displayHeight);

    this->allocateGPUMemory();
    this->recieveGpuTroubleShootData = new float[30];
    std::random_device rd;
    this->randomSeedGenerator.seed(rd());
    this->seedDistribution = std::uniform_real_distribution<float>(0,1);
};



Display::~Display() {
    std::cout<<"Display Deconstructing"<<std::endl;

    if(this->gpuImage)
        cudaFree(this->gpuImage);
    if(this->gpuCameraData)
        cudaFree(this->gpuCameraData);
    if(this->displayWindow)
        SDL_DestroyWindow(this->displayWindow);
    if(this->imageTexture)
        SDL_DestroyTexture(this->imageTexture);
}




void Display::updateDisplayParameters(Settings* settings) {
    this->displayWidth = settings->getGraphicsSettingsValue("ScreenResolutionX");
    this->displayHeight = settings->getGraphicsSettingsValue("ScreenResolutionY");

}



#pragma region GPU MEMORY
void Display::allocateWorldData(World* world) {
    float** worldDataAsArray = world->getWorldObjectDataAsArray();
    this->numOfWorldObjects = world->numOfObjects;
    cudaMalloc(&this->gpuWorldObjectData, this->numOfWorldObjects * sizeof(float*));
    for(size_t i = 0; i < this->numOfWorldObjects; i++) {
        float* objectData;
        cudaMalloc(&objectData, 30 * sizeof(float));
        cudaMemcpy(objectData, worldDataAsArray[i],30*sizeof(float),cudaMemcpyHostToDevice);
        cudaMemcpy(&this->gpuWorldObjectData[i],&objectData,sizeof(float*),cudaMemcpyHostToDevice);
    }
}



void Display::allocateGPUMemory() {
    cudaMalloc(&this->gpuTroubleShootData, 30*sizeof(float));
    cudaMalloc(&this->gpuCameraData, 8 * sizeof(float));
    cudaMalloc(&this->gpuImage, this->displayWidth * this->displayHeight * 3 * sizeof(unsigned char));
}

void Display::reAllocateGPUMemory() {
    cudaFree(this->gpuImage);
    cudaMalloc(&this->gpuCameraData, 8 * sizeof(float));
    cudaMalloc(&this->gpuImage, this->displayWidth * this->displayHeight * 3 * sizeof(unsigned char));
}

void Display::updateGPUData() {
    cudaMemcpy(this->gpuCameraData, this->viewCamera->cameraData, 8 * sizeof(float), cudaMemcpyHostToDevice);
}

void Display::copyImageData() {
    cudaMemcpy(this->gpuImage, this->imageData, this->displayWidth * this->displayHeight * 3 * sizeof(unsigned char),cudaMemcpyHostToDevice);
}
#pragma endregion




void Display::getPixelInfo(int x, int y) {
    int index = (y * this->displayWidth + x) * 3;
    std::cout << "Pixel (" << x << ", " << y << "): ( " << static_cast<int>(this->imageData[index]) << ", " << static_cast<int>(this->imageData[index+1]) << ", " << static_cast<int>(this->imageData[index+2]) << " )" << std::endl;
}

void Display::resetCursor() {
    SDL_WarpMouseInWindow(this->displayWindow, this->displayWidth / 2, this->displayHeight / 2);
    this->previousMousePosX = this->displayWidth/2;
    this->previousMousePosY = this->displayHeight/2;
    this->mousePosX = this->displayWidth/2;
    this->mousePosY = this->displayHeight/2;
}

/*
NEXT STEP IS TO REORGANIZE EVENTS INTO THE EVENT MANAGER CLASS
NEEDED QUITE BADLY

*/




void Display::renderScene() {
    this->renderImage();
    SDL_RenderClear(this->imageRenderer);
    SDL_UpdateTexture(this->imageTexture, NULL, this->imageData, this->displayWidth*3);
    SDL_RenderCopy(this->imageRenderer, this->imageTexture, NULL, NULL);
    SDL_RenderPresent(this->imageRenderer);
}


void Display::handleDisplaySizeChange(int newDisplayWidth, int newDisplayHeight) {
    this->displayWidth = newDisplayWidth;
    this->displayHeight = newDisplayHeight;
    delete[] this->imageData;
    this->imageData = new unsigned char[this->displayWidth * this->displayHeight * 3];
    SDL_DestroyTexture(this->imageTexture);
    this->imageTexture = SDL_CreateTexture(this->imageRenderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, this->displayWidth, this->displayHeight);
    cudaFree(this->gpuImage);
    cudaMalloc(&this->gpuImage, this->displayWidth * this->displayHeight * 3 * sizeof(unsigned char));

}







void Display::renderImage() {
    dim3 threads(16, 8);
    dim3 blocksPerGrid(ceil(displayWidth / static_cast<float>(threads.x)),
                        ceil(displayHeight / static_cast<float>(threads.y)));

    //Update GPU Data
    this->updateGPUData();

    int randomGenSeed = static_cast<int>(99999.0*this->seedDistribution(this->randomSeedGenerator));
    

    // Launch the CUDA kernel defined inside the class
    RayTracingKernel<<<blocksPerGrid, threads>>>(randomGenSeed, 1, displayWidth, displayHeight, this->numOfWorldObjects,this->gpuCameraData,this->gpuImage, this->gpuWorldObjectData,this->gpuTroubleShootData);

    cudaError_t kernelError = cudaGetLastError();
    if (kernelError != cudaSuccess) {
        std::cout << "CUDA kernel launch error: " << cudaGetErrorString(kernelError) << std::endl;
        cudaFree(this->gpuImage); // Free allocated memory
        return;
    }

    // Wait for kernel to finish
    cudaDeviceSynchronize();

    // Copy the processed image back from GPU to CPU
    cudaMemcpy(this->imageData, this->gpuImage, displayWidth * displayHeight * 3 * sizeof(unsigned char), cudaMemcpyDeviceToHost);

    //USED TO COPY TROUBLESHOOT DATA TO CPU
    // cudaMemcpy(this->recieveGpuTroubleShootData, this->gpuTroubleShootData,30*sizeof(float),cudaMemcpyDeviceToHost);
    // std::cout<<"----------------------"<<std::endl;
    // for(size_t i = 0; i<8; i++) {
    //     std::cout<<"Data Index: "<<i<<"   Data Value: "<<this->recieveGpuTroubleShootData[i]<<std::endl;
    // }
    // Free GPU memory
};


void Display::seriousRenderOfImage(int numOfFrames,int maxRayBounces) {
    dim3 threads(16, 8);
    dim3 blocksPerGrid(ceil(displayWidth / static_cast<float>(threads.x)),
                        ceil(displayHeight / static_cast<float>(threads.y)));

    float* imageDataArray = new float[this->displayWidth*this->displayHeight*3];
    float* gpuImageDataArray;
    cudaMalloc(&gpuImageDataArray,this->displayWidth*this->displayHeight*3*sizeof(float));

    for(int frame = 1; frame<=numOfFrames; frame++) {
        std::cout<<"Rendering Frame: "<<frame<<std::endl;
        //Update GPU Data
        this->updateGPUData();
        this->copyImageData();

        int randomGenSeed = static_cast<int>(99999.0*this->seedDistribution(this->randomSeedGenerator));
        
        
        // Launch the CUDA kernel defined inside the class
        DiffuseRender<<<blocksPerGrid, threads>>>(randomGenSeed,frame,maxRayBounces,displayWidth, displayHeight, this->numOfWorldObjects,this->gpuCameraData,gpuImageDataArray, this->gpuWorldObjectData,this->gpuTroubleShootData);
        
        cudaError_t kernelError = cudaGetLastError();
        if (kernelError != cudaSuccess) {
            std::cout << "CUDA kernel launch error: " << cudaGetErrorString(kernelError) << std::endl;
            cudaFree(this->gpuImage); // Free allocated memory
            return;
        }

        // Wait for kernel to finish
        cudaDeviceSynchronize();

        updateImage<<<blocksPerGrid, threads>>>(frame,displayWidth, displayHeight,gpuImageDataArray,this->gpuImage,this->gpuTroubleShootData);
        kernelError = cudaGetLastError();
        if (kernelError != cudaSuccess) {
            std::cout << "CUDA kernel launch error: " << cudaGetErrorString(kernelError) << std::endl;
            cudaFree(this->gpuImage); // Free allocated memory
            return;
        }

        // Wait for kernel to finish
        cudaDeviceSynchronize();

        // Copy the processed image back from GPU to CPU
        cudaMemcpy(this->imageData, this->gpuImage, displayWidth * displayHeight * 3 * sizeof(unsigned char), cudaMemcpyDeviceToHost);

        //Updates DISPLAYED Image
        SDL_RenderClear(this->imageRenderer);
        SDL_UpdateTexture(this->imageTexture, NULL, this->imageData, this->displayWidth*3);
        SDL_RenderCopy(this->imageRenderer, this->imageTexture, NULL, NULL);
        SDL_RenderPresent(this->imageRenderer);
    }
}



void Display::customResolutionRender(std::string fileName, int numOfFrames,int maxRayBounces, int resolutionX, int resolutionY) {
    dim3 threads(16, 8);
    dim3 blocksPerGrid(ceil(resolutionX / static_cast<float>(threads.x)),
                        ceil(resolutionY / static_cast<float>(threads.y)));

    float* imageDataArray = new float[resolutionX*resolutionY*3];
    float* gpuImageDataArray;
    cudaMalloc(&gpuImageDataArray,resolutionX*resolutionY*3*sizeof(float));

    unsigned char* customResImageData = new unsigned char[resolutionX*resolutionY*3];
    unsigned char* customResGPUImageData;
    cudaMalloc(&customResGPUImageData,resolutionX*resolutionY*3*sizeof(unsigned char));

    cudaError_t kernelError;

    for(int frame = 1; frame<=numOfFrames; frame++) {
        std::cout<<"Rendering Frame: "<<frame<<std::endl;
        //Update GPU Data
        this->updateGPUData();
        this->copyImageData();

        int randomGenSeed = static_cast<int>(99999.0*this->seedDistribution(this->randomSeedGenerator));
        
        
        // Launch the CUDA kernel defined inside the class
        DiffuseRender<<<blocksPerGrid, threads>>>(randomGenSeed,frame,maxRayBounces,resolutionX, resolutionY, this->numOfWorldObjects,this->gpuCameraData,gpuImageDataArray, this->gpuWorldObjectData,this->gpuTroubleShootData);
        
        kernelError = cudaGetLastError();
        if (kernelError != cudaSuccess) {
            std::cout << "CUDA kernel launch error: " << cudaGetErrorString(kernelError) << std::endl;
            cudaFree(this->gpuImage); // Free allocated memory
            return;
        }

        // Wait for kernel to finish
        cudaDeviceSynchronize();
    }
    updateImage<<<blocksPerGrid, threads>>>(numOfFrames,resolutionX, resolutionY,gpuImageDataArray,customResGPUImageData,this->gpuTroubleShootData);
        kernelError = cudaGetLastError();
        if (kernelError != cudaSuccess) {
            std::cout << "CUDA kernel launch error: " << cudaGetErrorString(kernelError) << std::endl;
            cudaFree(this->gpuImage); // Free allocated memory
            return;
        }

    // Wait for kernel to finish
    cudaDeviceSynchronize();

    // Copy the processed image back from GPU to CPU
    cudaMemcpy(customResImageData, customResGPUImageData, resolutionX*resolutionY * 3 * sizeof(unsigned char), cudaMemcpyDeviceToHost);

    std::cout<<"saving image..."<<std::endl;
    // Write image data to BMP file
    SDL_Surface* surface = SDL_CreateRGBSurfaceFrom(customResImageData, resolutionX, resolutionY, 24, resolutionX * 3, 0xFF0000, 0x00FF00, 0x0000FF, 0);
    std::string filePath = "./Saved Images/" +fileName + ".bmp";
    std::cout<<"FilePath: "<<filePath<<std::endl;
    SDL_SaveBMP(surface,filePath.c_str());
}


void Display::saveImage(const std::string& fileName) {
    std::cout<<"saving image..."<<std::endl;
    // Write image data to BMP file
    SDL_Surface* surface = SDL_CreateRGBSurface(0, displayWidth, displayHeight, 32, 0, 0, 0, 0);
    SDL_RenderReadPixels(this->imageRenderer,NULL,SDL_PIXELFORMAT_ARGB8888, surface->pixels, surface->pitch);

    std::string filePath = "./Saved Images/" +fileName + ".bmp";
    std::cout<<"FilePath: "<<filePath<<std::endl;
    SDL_SaveBMP(surface,filePath.c_str());
    
}

