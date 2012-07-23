#include <QtGui/QApplication>
#include <QtCore/QCoreApplication>
#include <QtCore/QTimer>
#include <QCursor>

#include "mainwindow.h"

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <BlobResult.h>
#include <blob.h>
#include "squares.cpp"
#include "matching.cpp"

using namespace cv;

CvCapture* capture;
CvCapture* capture1;
CvMemStorage* storage = 0;
CvMemStorage* storage1 = 0;

//drawSquares( img, findSquares4( img, storage ) );

struct finger {
    CvPoint tip;
    CvPoint valley;
};

typedef struct finger FINGER;



enum ColorsForThresh {
        Blue,
        Red,
        Hand
};

FINGER fingers[20];

IplImage *GetThresholdedImage(IplImage* img, int colorForThresh, CvScalar blue_l, CvScalar blue_h, CvScalar red_l, CvScalar red_h)
{

        IplImage* imgHSV=cvCreateImage(cvGetSize(img),8,3);
        cvCvtColor(img,imgHSV,CV_BGR2HSV);

        IplImage* imgThreshed=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
        IplImage* imgThreshed1=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);

        if(colorForThresh==Blue) cvInRangeS(imgHSV,blue_l,blue_h,imgThreshed);

        else if(colorForThresh==Red) cvInRangeS(imgHSV,red_l,red_h,imgThreshed);

        else if(colorForThresh==Hand) {
                //cvInRangeS(imgHSV,cvScalar(3,30,20),cvScalar(18,255,260),imgThreshed);
                cvInRangeS(imgHSV,cvScalar(0,30,40),cvScalar(20,150,255),imgThreshed);
                //cvInRangeS(imgHSV,cvScalar(165,30,22),cvScalar(180,250,240),imgThreshed1);
                //cvAdd(imgThreshed,imgThreshed1,imgThreshed);
        }

        cvReleaseImage(&imgHSV);

        return imgThreshed;

}



float getDistance(int x1,int y1,int x2,int y2) {
        double dx=(x1-x2);
        double dy=(y1-y2);
        dx*=dx;
        dy*=dy;
        return (sqrt(dx+dy));
}

float getColorDistance(uchar B1,uchar G1, uchar R1,uchar B2,uchar G2, uchar R2) {
    double db=(B1-B2);
    double dg=(G1-G2);
    double dr=(R1-R2);

    db*=db;
    dg*=dg;
    dr*=dr;

    return sqrt(db+dg+dr);
}

void printBool(bool a) {
        if(a==true) printf(" true ");
        else printf(" false ");
}

void drawBall(IplImage* img,int x,int y,int radius) {
        cvCircle(img,cvPoint(x,y),radius,cvScalar(255,255,0),-1);
}

void drawCircle(IplImage* img,int x, int y, CvScalar color) {
        cvCircle(img,cvPoint(x,y),3,color,3);
}


void MainWindow::CalibBlue() {
    capture=cvCaptureFromCAM(0);
    float hue=0,hueh=0,huel=300,sat=0,val=0;
    IplImage* frame=NULL;
    int i=0;
    int c,t=0;

    int hue_l,hue_h;
    int sat_l,sat_h;
    int val_l,val_h;

    CvScalar s;
    frame=cvQueryFrame(capture);
    frame=cvQueryFrame(capture);
    frame=cvQueryFrame(capture);
    while(i++<300)  {
        frame=cvQueryFrame(capture);
        cvFlip(frame,frame,1);


        //cvCanny(frame,frame,10,100,3);

        //cvShowImage("frame_calib_blue",frame);


        cvCvtColor(frame,frame,CV_BGR2HSV);
        s=cvGet2D(frame,240,320);
        cvCvtColor(frame,frame,CV_HSV2BGR);
        cvLine(frame,cvPoint(310,240),cvPoint(330,240),cvScalar(255,255,255));
        cvLine(frame,cvPoint(320,230),cvPoint(320,250),cvScalar(255,255,255));
        cvShowImage("frame_calib_blue",frame);




        if(i>=100) {
            if(t==0) {
                qDebug("CALIBRATION BEGUN");
                t=1;
            }
            hue+=s.val[0];
            if(huel>s.val[0]) huel=s.val[0];
            if(hueh<s.val[0]) hueh=s.val[0];

            sat+=s.val[1];
            val+=s.val[2];
        }
        c=cvWaitKey(5);
        if(c==27) break;
    }
    cvDestroyWindow("frame_calib_blue");
    hue/=200;
    sat/=200;
    val/=200;


    hue_l=hue-10;
    if(hue_l<0) hue_l=0;
    hue_h=hue+10;
    if(hue_h>290) hue_h=290;

    blue_h_h=hueh+5;
    blue_h_l=huel-5;



    sat_l=sat-80;
    if(sat_l<0) sat_l=0;
    sat_h=sat+80;
    if(sat_h>290) sat_h=290;
    blue_s_h=sat_h;
    blue_s_l=sat_l;



    val_l=val-80;
    if(val_l<0) val_l=0;
    val_h=val+80;
    if(val_h>290) val_h=290;
    blue_v_h=val_h;
    blue_v_l=val_l;

    ("CALIBRATION DONE.\nBLUE: %d,%d    %d,%d    %d,%d",blue_h_l,blue_h_h,blue_s_l,blue_s_h,blue_v_l,blue_v_h);
}

int MainWindow::match(int a, int b) {
    qDebug("%d %d",a,b);
    return func(a,b);

}

void MainWindow::CalibRed() {
    capture=cvCaptureFromCAM(0);
    float hue=0,sat=0,val=0,hueh=0,huel=300;
    IplImage* frame=NULL;
    int i=0;
    int c;
    CvScalar s;
    int t=0;
    frame=cvQueryFrame(capture);
    frame=cvQueryFrame(capture);
    frame=cvQueryFrame(capture);
    while(i++<300)  {
        frame=cvQueryFrame(capture);
        cvFlip(frame,frame,1);
        cvCvtColor(frame,frame,CV_BGR2HSV);
        s=cvGet2D(frame,240,320);
        cvCvtColor(frame,frame,CV_HSV2BGR);
        cvLine(frame,cvPoint(310,240),cvPoint(330,240),cvScalar(255,255,255));
        cvLine(frame,cvPoint(320,230),cvPoint(320,250),cvScalar(255,255,255));
        cvShowImage("frame_calib_red",frame);



        if(i>=100) {
            if(t==0) {
                qDebug("CALIBRATION BEGUN");
                t=1;
            }
            if(huel>s.val[0]) huel=s.val[0];
            if(hueh<s.val[0]) hueh=s.val[0];
            hue+=s.val[0];
            sat+=s.val[1];
            val+=s.val[2];
        }
        c=cvWaitKey(5);
        if(c==27) break;
    }
    cvDestroyWindow("frame_calib_red");
    hue/=200;
    sat/=200;
    val/=200;

    int hue_l,hue_h;
    hue_l=hue-10;
    if(hue_l<0) hue_l=0;
    hue_h=hue+10;
    if(hue_h>290) hue_h=290;
    red_h_h=hueh+5;
    red_h_l=huel-5;


    int sat_l,sat_h;
    sat_l=sat-80;
    if(sat_l<0) sat_l=0;
    sat_h=sat+80;
    if(sat_h>290) sat_h=290;
    red_s_h=sat_h;
    red_s_l=sat_l;

    /*
    int val_l,val_h;
    val_l=val-80;
    if(val_l<0) val_l=0;
    val_h=val+80;
    if(val_h>290) val_h=290;
    red_v_h=val_h;
    red_v_l=val_l;
*/
    qDebug("red: %f %f",hue,sat);
}

CvPoint getCenterBlob(IplImage* img) {
    int posX,posY;
    CBlobResult blobs;
    CBlob blob;

    blobs=CBlobResult(img,NULL,0,true);    
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 500 );
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, 9000 );

    //("%d",blobs.GetNumBlobs());

    blobs.GetNthBlob(CBlobGetArea(),blobs.GetNumBlobs() - 1,blob);


    posX=((int)blob.MaxX() + (int)blob.MinX())/2;
    posY=((int)blob.MaxY() + (int)blob.MinY())/2;

    return cvPoint(posX,posY);
}

CBlobResult getCenterBlobs4(IplImage* img) {

    CBlobResult blobs;
    blobs=CBlobResult(img,NULL,0,true);
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 1500 );
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, 9000 );

    //("%d",blobs.GetNumBlobs());
    return blobs;

}

CvPoint getCenterMoment(IplImage* img) {
    double moment10,moment01;
    double area1;
    int posX,posY;
    CvMoments *moments;

    moments = (CvMoments*)malloc(sizeof(CvMoments));


    cvMoments(img, moments, 1);


    moment10 = cvGetSpatialMoment(moments, 1, 0);
    moment01 = cvGetSpatialMoment(moments, 0, 1);
    area1= cvGetCentralMoment(moments, 0, 0);

    posX = moment10/area1;
    posY = moment01/area1;

    return cvPoint(posX,posY);


}


void getCenterCanny(IplImage* img) {
       //assert( img->width%2 == 0 && img->height%2 == 0);
       IplImage* out = cvCreateImage(cvSize(img->width/2,img->height/2),img->depth,img->nChannels);
       cvPyrDown( img, out );
       cvCanny( out, out, 10, 100, 3 );
       cvShowImage("canny",out);
}

IplImage* transposeImage(IplImage* image, int angle) {

  IplImage *rotated = cvCreateImage(cvSize(image->height,image->width), IPL_DEPTH_8U,image->nChannels);

  CvPoint2D32f center;

  float center_val = (float)((image->width)-1) / 2;
  center.x = center_val;
  center.y = center_val;
  CvMat *mapMatrix = cvCreateMat( 2, 3, CV_32FC1 );

  cv2DRotationMatrix(center, angle, 1.0, mapMatrix);
  cvWarpAffine(image, rotated, mapMatrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

  cvReleaseMat(&mapMatrix);

  return rotated;
}

IplImage* rotateImage(const IplImage *src, float angleDegrees)
{
    // Create a map_matrix, where the left 2x2 matrix
    // is the transform and the right 2x1 is the dimensions.
    float m[6];
    CvMat M = cvMat(2, 3, CV_32F, m);
    int w = src->width;
    int h = src->height;
    float angleRadians = angleDegrees * ((float)CV_PI / 180.0f);
    m[0] = (float)( cos(angleRadians) );
    m[1] = (float)( sin(angleRadians) );
    m[3] = -m[1];
    m[4] = m[0];
    m[2] = w*0.5f;
    m[5] = h*0.5f;

    // Make a spare image for the result
    CvSize sizeRotated;
    sizeRotated.width = cvRound(w);
    sizeRotated.height = cvRound(h);

    // Rotate
    IplImage *imageRotated = cvCreateImage( sizeRotated,
        src->depth, src->nChannels );

    // Transform the image
    cvGetQuadrangleSubPix( src, imageRotated, &M);

    return imageRotated;
}


int MainWindow::beginProjection() {

    capture=cvCaptureFromCAM(0);

    //keystroke
    int c;

    if(!capture)
    {
            printf("Could not initialize capturing...\n");
            return -1;
    }

    IplImage* frame=NULL;
    IplImage* overlay=NULL;


    IplImage* src = cvLoadImage("img.png", 1);

    /*
    IplImage* src=cvCreateImage(cvSize(srcSmall->width*2,srcSmall->height*2),srcSmall->depth, srcSmall->nChannels);
    cvZero(src);
    cvSetImageROI(src,cvRect( (src->width/2)-(srcSmall->width/2) , (src->height/2)-(srcSmall->height/2) , srcSmall->width,srcSmall->height));
    cvCopy(srcSmall,src);
    cvResetImageROI(src);
    */

    IplImage* tempImg;

    int depth;

    cv::Size size;
    CvScalar s;


    frame=cvQueryFrame(capture);
    frame=cvQueryFrame(capture);

    overlay=cvCloneImage(src);

    size = cvGetSize(frame);
    depth = frame->depth;


    if(!frame) return -1;

    CvPoint *sqFinal1;
    float m[6];
    CvMat M=cvMat(2,3,CV_32F,m);
    int w=src->width;
    int h=src->height;



    tempImg=cvCreateImage(cvSize((int)((frame->width*33)/100),(int)((frame->height*33)/100)),frame->depth, frame->nChannels );

    int squareOrder[4];

    double angle,dy,dx;
    CvRect overlayRect;

    int len_square,bre_square;

    while(1==1) {

        cvFlip(frame,frame,1);

        frame=cvQueryFrame(capture);


        cvCvtColor(frame,frame,CV_BGR2HSV);
        s=cvGet2D(frame,240,320); //for debugging
        cvResize(frame,tempImg);
        cvCvtColor(frame,frame,CV_HSV2BGR);



        storage = cvCreateMemStorage(0);
        sqFinal1=drawSquares( frame, findSquares4( frame, storage ));
        cvClearMemStorage(storage);


        //draw white circles in the 4 squares
        cvCircle(frame,cvPoint(sqFinal1[0].x,sqFinal1[0].y),10,cvScalar(255,255,255));
        cvCircle(frame,cvPoint(sqFinal1[1].x,sqFinal1[1].y),10,cvScalar(255,255,255));
        cvCircle(frame,cvPoint(sqFinal1[2].x,sqFinal1[2].y),10,cvScalar(255,255,255));
        cvCircle(frame,cvPoint(sqFinal1[3].x,sqFinal1[3].y),10,cvScalar(255,255,255));

        //Correct the order of squares
        {
        if(sqFinal1[0].x>sqFinal1[1].x) {
            if(sqFinal1[0].y>sqFinal1[3].y) {
                squareOrder[0]=2;
                squareOrder[1]=3;
                squareOrder[2]=0;
                squareOrder[3]=1;
            }
            else {
                squareOrder[0]=1;
                squareOrder[1]=0;
                squareOrder[2]=3;
                squareOrder[3]=2;
            }
        }
        else {
            if(sqFinal1[0].y>sqFinal1[3].y) {
                squareOrder[0]=3;
                squareOrder[1]=2;
                squareOrder[2]=1;
                squareOrder[3]=0;
            }
            else {
                squareOrder[0]=0;
                squareOrder[1]=1;
                squareOrder[2]=2;
                squareOrder[3]=3;
            }
        }
        }

        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);

        /*
        for(jLoop=0;jLoop<4;jLoop++) {
            sprintf(buffer,"%d",jLoop);
            cvPutText(frame,buffer,sqFinal1[squareOrder[jLoop]],&font,cvScalar(0,255,0));
        }
        */

        if((sqFinal1[1].x-sqFinal1[squareOrder[0]].x)!=0) {
            dy=(double)(sqFinal1[squareOrder[1]].y)-double(sqFinal1[squareOrder[0]].y);
            dx=(double)(sqFinal1[squareOrder[1]].x)-double(sqFinal1[squareOrder[0]].x);
            angle=atan(dy/dx);
        }
        else
            angle=(CV_PI/2);

        angle*=(180.0/CV_PI); //in degrees now
        cvNamedWindow("rot",CV_WINDOW_AUTOSIZE);

        len_square=(int) getDistance(sqFinal1[squareOrder[0]].x,sqFinal1[squareOrder[0]].y,sqFinal1[squareOrder[1]].x,sqFinal1[squareOrder[1]].y);
        bre_square=(int) getDistance(sqFinal1[squareOrder[2]].x,sqFinal1[squareOrder[2]].y,sqFinal1[squareOrder[1]].x,sqFinal1[squareOrder[1]].y);

        if(len_square>0 && bre_square>0) {
            //angle,len_square

            overlay=rotateImage(src,angle);

            overlayRect=cvRect(
                        (sqFinal1[squareOrder[0]].x+sqFinal1[squareOrder[1]].x+sqFinal1[squareOrder[2]].x+sqFinal1[squareOrder[3]].x)/4-(len_square/2),
                          (sqFinal1[squareOrder[0]].y+sqFinal1[squareOrder[1]].y+sqFinal1[squareOrder[2]].y+sqFinal1[squareOrder[3]].y)/4-(bre_square/2),
                          //overlay->width,overlay->height
                         len_square,bre_square
                        );



            cvSetImageROI(frame,overlayRect);
            //cvResize(overlay,frame);
            qDebug("%d %d %d",frame->height, overlayRect.height, overlay->height);

            //cvAdd(frame,overlay,frame,NULL);

            //cvShowImage("rot",overlay);
            cvResize(overlay,frame,1);


            //cvCopy(frame,overlay);
            cvResetImageROI(frame);

        }
        cvShowImage("frame",frame);
        cvNamedWindow("frame",0);

        c=cvWaitKey(1);

        if((char)c==27 || MainWindow::fin==1) {
            MainWindow::fin==0;
           cvDestroyAllWindows();
           break;
       }

    }

    cvReleaseCapture(&capture);
    return 0;
}




int MainWindow::beginHand() {

    capture=cvCaptureFromCAM(0);

    int c;

    if(!capture)
    {
            printf("Could not initialize capturing...\n");
            return -1;
    }

    IplImage* frame=NULL;

    IplImage *hand;

    int depth;

    CvScalar s;

    int tx,ty;
    int diag;

    float blurFactor=1;

    frame=cvQueryFrame(capture);
    frame=cvQueryFrame(capture);

    CvSize size = cvGetSize(frame);
    depth = frame->depth;

    if(!frame) return -1;
    CvScalar scalar_temp=cvScalar(0);
    CvSeq* hull;
    CvSeq* defect;
    CvPoint depthPoints[20];
    CvPoint tipPoints[20];

    while(1==1) {

        cvFlip(frame,frame,1);

        frame=cvQueryFrame(capture);

        cvCvtColor(frame,frame,CV_BGR2HSV);
        s=cvGet2D(frame,240,320); //for debugging
        cvCvtColor(frame,frame,CV_HSV2BGR);



        hand=GetThresholdedImage(frame,Hand, scalar_temp,scalar_temp,scalar_temp,scalar_temp);

        cvSmooth(hand,hand,CV_MEDIAN,5);
        cvSmooth(hand,hand,CV_MEDIAN,9);
        cvSmooth(hand,hand,CV_MEDIAN,9);

        CvPoint tempP;
       // cvShowImage("hand",hand);
       cvNamedWindow("frame",0);

       cvErode(hand,hand);
       cvDilate(hand,hand);

       //cvCanny(hand,hand,190,190,5);
       //cvShowImage("hand1",hand);

       //cvLaplace(hand,edges,3);
       //cvSobel(hand,edges,1,1,3);
       //cvConvertScaleAbs(edges,hand,4,0);

       cvSmooth(hand,hand,CV_MEDIAN,7);
       cvSmooth(hand,hand,CV_GAUSSIAN);

       //cvDilate(hand,hand);
        //cvErode(hand,hand);

        //cvSmooth(hand,hand,CV_MEDIAN,7);

       tempP=getCenterBlob(hand);

        cvCircle(frame,tempP,4,cvScalar(0,0,255),1);


       storage=cvCreateMemStorage(0);
       storage1=cvCreateMemStorage(0);

       CvSeq* contour=0;
       cvFindContours( hand, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

       cvZero( frame );
       double cArea=0;
       double tempArea;
       CvSeq* bigContour=0;
       for( ; contour != 0; contour = contour->h_next )
       {
           tempArea=fabs(cvContourArea(contour));
           if(tempArea>cArea) {
               cArea=tempArea;
               bigContour=contour;
           }

       //cvDrawContours( frame, contour, CV_RGB(0,255,0),CV_RGB(255,255,0), -1, 1, 8 );
       //contour->
       }
       cvDrawContours( frame, bigContour, CV_RGB(0,255,0),CV_RGB(255,255,0), -1, 1, 8 );

      CvRect boundRect = cvBoundingRect(bigContour);
      cvRectangle(frame,cvPoint(boundRect.x,boundRect.y),cvPoint(boundRect.x+boundRect.width,boundRect.y+boundRect.height),cvScalar(255,255,255));
     //cvLine(frame,cvPoint(310,240),cvPoint(330,240),cvScalar(255,255,255));

      hull = cvConvexHull2(bigContour,0,CV_CLOCKWISE,0);
      defect=cvConvexityDefects(bigContour,hull,storage1);

      qDebug("%d",defect->total);


      CvFont font;
      double hScale=1.0;
      double vScale=1.0;
      int    lineWidth=1;
      cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);



      CvConvexityDefect* defectArray;
      CvPoint depthTemp, tipTemp, depthTempj, endTempj, startTempj, tipTempj,cdepthpoint;
     int i;

         int nomdef = defect->total;
          defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*nomdef);
          cvCvtSeqToArray (defect, defectArray, CV_WHOLE_SEQ);
          int idx=0;

          for(i=0; i<nomdef;i++)
          {
              //cvCircle( frame, *(defectArray[i].end),5, CV_RGB(255,0,0), -1, 8,0);
              //cvCircle( frame, *(defectArray[i].start), 5, CV_RGB(0,0,255), -1, 8,0);
              depthTemp=*(defectArray[i].depth_point);
              tipTemp=*(defectArray[i].end);
              int j,test=0;

              char strIdx[10];
              int ptl=0;
              if(i==0)  cdepthpoint = *(defectArray[1].depth_point);
              else cdepthpoint = *(defectArray[0].depth_point);
              for(j=0;j<nomdef;j++) {
                  if(j==i) continue;
                  depthTempj=*(defectArray[j].depth_point);
                  endTempj=*(defectArray[j].end);
                  startTempj=*(defectArray[j].start);                  

                  if(getDistance(depthTemp.x,depthTemp.y,endTempj.x,endTempj.y) < 20 ||
                     getDistance(depthTemp.x,depthTemp.y,startTempj.x,startTempj.y) < 20
                          ) {
                      test=1;
                      break;
                  }
              }

              if(!test) {
                  ptl++;
                  cvCircle( frame, *(defectArray[i].depth_point), 5, CV_RGB(0,255,255), -1, 8,0); //valleys
                  sprintf(strIdx,"%d",idx);
                  //cvPutText(frame,strIdx,*(defectArray[i].depth_point),&font,CV_RGB(255,0,0));
              }


              test=0;
              for(j=i+1;j<nomdef;j++) {
                  tipTempj=*(defectArray[j].end);
                  if(getDistance(tipTemp.x,tipTemp.y,tipTempj.x,tipTempj.y) < 20) {
                      test=1; break;
                  }
              }

              if(!test) {
                  ptl++;
                  cvCircle( frame, *(defectArray[i].end), 5, CV_RGB(0,0,255), -1, 8,0); //tips
                  sprintf(strIdx,"%d",idx);
                  //cvPutText(frame,strIdx,*(defectArray[i].end),&font,CV_RGB(255,0,0));
              }

             /* if(ptl==2) {
                  fingers[idx].tip=*(defectArray[i].end);
                  fingers[idx].valley=*(defectArray[i].depth_point);

                  cvLine(frame,fingers[idx].tip,fingers[idx].valley,CV_RGB(255,0,0));
              }*/

              depthTemp=*(defectArray[i].depth_point);
              depthTemp=cvPoint(depthTemp.x-boundRect.x,depthTemp.y-boundRect.y);

              tipTemp=*(defectArray[i].start);
              tipTemp=cvPoint(tipTemp.x-boundRect.x,tipTemp.y-boundRect.y);


               idx++;

              if(i>20) break;
          }
          //j++;

          //draw on




        //qDebug("%f %f %f\n",s.val[0],s.val[1],s.val[2]);


       cvShowImage("frame",frame);

       c=cvWaitKey(1);

       if((char)c==27 || MainWindow::fin==1) {
           MainWindow::fin==0;
           cvDestroyAllWindows();
           break;
       }

    }

    cvReleaseCapture(&capture);
    return 0;
}

void MainWindow::beginDrawing() {
    capture = cvCaptureFromCAM(0);
    if(!capture)
    {
            printf("Could not initialize capturing...\n");
    }

    IplImage* frame=NULL;
    IplImage *slate;

    IplImage *blue;

    CvScalar blue_l=cvScalar(blue_h_l,blue_s_l, blue_v_l);
    CvScalar blue_h=cvScalar(blue_h_h,blue_s_h, blue_v_h);
    CvScalar red_l=cvScalar(red_h_l,red_s_l, red_v_l);
    CvScalar red_h=cvScalar(red_h_h,red_s_h, red_v_h);
    int lastXBlue;
    int lastYBlue;
    int c;

    frame=cvQueryFrame(capture);
    slate = cvCloneImage(frame);
    frame=cvQueryFrame(capture);
    cvZero(slate);

    CvPoint tempP;
    static int posXBlue;
    static int posYBlue;

    while(1) {
        frame=cvQueryFrame(capture);

        blue=GetThresholdedImage(frame,Blue, blue_l, blue_h, red_l, red_h);
        cvSmooth(blue,blue,CV_MEDIAN,13);

        lastXBlue = posXBlue;
        lastYBlue = posYBlue;

        tempP=getCenterBlob(blue);
        //tempP=getCenterMoment(blue);
        posXBlue=tempP.x;
        posYBlue=tempP.y;

        cvCircle(slate,cvPoint(posXBlue,posYBlue),3,cvScalar(0,0,255),7);

        cvAdd(frame,slate,frame);
        cvShowImage("Frame",frame);

        c=cvWaitKey(1);

        if((char)c==27 || MainWindow::fin==1) {
            MainWindow::fin==0;
            cvDestroyAllWindows();
            break;
        }
    }


}

int MainWindow::beginThimble() {

    capture=cvCaptureFromCAM(0);

    int c;

    if(!capture)
    {
            printf("Could not initialize capturing...\n");
            return -1;
    }

    IplImage* frame=NULL;

    IplImage *blue, *red;
    int depth;


    cv::Size size;
    CvScalar s;

    int tx,ty;
    int diag;

    float blurFactor=1;

    int posXBall=320,posYBall=240;
    int posXGrip,posYGrip;

    bool testHold;
    bool testGrip;
    int GRIP_THRESHOLD=80;
    int radius=20;

    frame=cvQueryFrame(capture);
    frame=cvQueryFrame(capture);

    size = cvGetSize(frame);
    depth = frame->depth;

    blue = cvCreateImage(size, depth, 1);
    cvZero(blue);

    red = cvCreateImage(size, depth, 1);
    cvZero(red);

    if(!frame) return -1;

    bool hold=false,hold1=false,hold2=false,hold3=false,hold4=false,hold5=false;
    bool grip=false,grip1=false,grip2=false,grip3=false,grip4=false,grip5=false;
    bool gripa=false;


    static int posXBlue = 0;
    static int posYBlue = 0;
    static int posXRed = 0;
    static int posYRed = 0;

    int lastXRed;
    int lastYRed;

    int lastXBlue;
    int lastYBlue;
    float distance,gripDistance;

    CvScalar blue_l=cvScalar(blue_h_l,blue_s_l, blue_v_l);
    CvScalar blue_h=cvScalar(blue_h_h,blue_s_h, blue_v_h);
    CvScalar red_l=cvScalar(red_h_l,red_s_l, red_v_l);
    CvScalar red_h=cvScalar(red_h_h,red_s_h, red_v_h);

    CvPoint tempP;



    while(1==1) {

        hold5=hold4;   hold4=hold3;    hold3=hold2; hold2=hold1;   hold1=hold;
        grip5=grip4; grip4=grip3; grip3=grip2; grip2=grip1; grip1=grip;
        cvFlip(frame,frame,1);


        frame=cvQueryFrame(capture);

        cvCvtColor(frame,frame,CV_BGR2HSV);
        s=cvGet2D(frame,240,320); //for debugging
        cvCvtColor(frame,frame,CV_HSV2BGR);


        /*---2 images to be checked---*/
        blue=GetThresholdedImage(frame,Blue, blue_l, blue_h, red_l, red_h);
        red=GetThresholdedImage(frame,Red, blue_l, blue_h, red_l, red_h);
        cvSmooth(red,red,CV_MEDIAN,13);
        cvSmooth(blue,blue,CV_MEDIAN,13);

        lastXBlue = posXBlue;
        lastYBlue = posYBlue;

        tempP=getCenterBlob(blue);
        //tempP=getCenterMoment(blue);
        posXBlue=tempP.x;
        posYBlue=tempP.y;
        cvCircle(frame,cvPoint(posXBlue,posYBlue),5,cvScalar(255,255,255),3);

        posXBlue=(blurFactor*posXBlue)+((1.0-blurFactor)*lastXBlue);
        posYBlue=(blurFactor*posYBlue)+((1.0-blurFactor)*lastYBlue);
        //---^^^^ CENTER OF BLUE ^^^^^^----



        lastXRed = posXRed;
        lastYRed = posYRed;

        tempP=getCenterBlob(red);
        //tempP=getCenterMoment(red);
        posXRed=tempP.x;
        posYRed=tempP.y;

        cvCircle(frame,cvPoint(posXRed,posYRed),5,cvScalar(255,255,255),3);

        posXRed=(blurFactor*posXRed)+((1.0-blurFactor)*lastXRed);
        posYRed=(blurFactor*posYRed)+((1.0-blurFactor)*lastYRed);
        //^^^^---CENTER OF RED-----




        posXGrip=(posXRed+posXBlue)/2;
        posYGrip=(posYRed+posYBlue)/2;


        drawBall(frame,posXBall,posYBall,radius);
        distance=getDistance(posXRed,posYRed,posXBlue,posYBlue);
        hold=(distance<=GRIP_THRESHOLD);
        testHold=(hold || hold1 || hold2 || hold3);

        gripDistance=getDistance(posXGrip,posYGrip,posXBall,posYBall);
        grip=(gripDistance<radius);
        testGrip=(grip || grip1 || grip2 || grip3);


        if(gripa && testGrip && hold) {
            drawCircle(frame,posXGrip,posYGrip,cvScalar(0,0,255,0));
            gripa=true;
        }
        else if(testGrip && hold1==false && hold==true)
        {
            gripa=true;
            drawCircle(frame,posXGrip,posYGrip,cvScalar(0,0,255,0));
        }
        else {
            gripa=false;
            printBool((gripDistance<radius));
            if(hold) drawCircle(frame,posXGrip,posYGrip,cvScalar(255,0,0,0));
        }

        if(gripa) {
            posXBall=posXGrip;
            posYBall=posYGrip;
            //QCursor::setPos(640-posXBall,posYBall);
        }



       if(distance>GRIP_THRESHOLD)	{
           printf("\nFar");
           qDebug("Not drawing");

       }
       else {
           printf("\nCLOSE");
       }


/*
       blueblobs=getCenterBlobs4(blue);
       numBlobs=blueblobs.GetNumBlobs();
       tx=ty=0;
       for(int i=0;i<numBlobs;i++) {
           if(numBlobs!=4) break;
           tempBlob=blueblobs.GetBlob(i);
           tempBlob.FillBlob(frame,cvScalar(255,0,0));
           squarePt[i]=cvPoint((tempBlob.MaxX()+tempBlob.MinX())/2,(tempBlob.MaxY()+tempBlob.MinY())/2);
           tx+=(tempBlob.MaxX()+tempBlob.MinX())/2;
           ty+=(tempBlob.MaxY()+tempBlob.MinY())/2;
           qDebug("%lf",tempBlob.Area());
       }

       tx/=4;
       ty/=4;

       diag=getDistance(tx,ty,squarePt[0].x,squarePt[0].y);
       diag=diag*2;
*/


       //cvShowImage("videoBlue",blue);
       //cvShowImage("videoRed",red);

       cvLine(frame,cvPoint(310,240),cvPoint(330,240),cvScalar(255,255,255));

       //qDebug("%f %f %f\n",s.val[0],s.val[1],s.val[2]);

       //cvFlip(frame,frame,1);
       cvNamedWindow("frame",0);


       cvShowImage("frame",frame);



       c=cvWaitKey(1);

       if((char)c==27 || MainWindow::fin==1) {
           qDebug("problem here");
           MainWindow::fin==0;
           cvDestroyAllWindows();
           break;
       }



    }

    cvReleaseCapture(&capture);
    return 0;
}

int MainWindow::beginGreen(int thresh) {
    capture=cvCaptureFromAVI("C:\\green6.m2t");

    int c;

    if(!capture)
    {
            printf("Could not initialize capturing...\n");
            return -1;
    }

    IplImage* frame;
    IplImage* tempFrame;


    frame=cvQueryFrame(capture);
    tempFrame=cvCloneImage(frame);

    int w,h,step;
    w=frame->width;
    h=frame->height;
    step=frame->widthStep;
    int channels=frame->nChannels;
    int i,j,c1=0;

    uchar *data;
    uchar *data1; //bgr
    uchar H,S,V,B,G,R;

    CvVideoWriter *writer = 0;
    int fps=(int) cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
    int frames=(int) cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_COUNT);

int aaa=1;
    qDebug("%d frames at %d fps",frames,fps);
    int isColor = 1;

    writer=cvCreateVideoWriter("out.avi",CV_FOURCC('D','I','V','X'),fps,cvSize(w,h),isColor);

    while(1) {
        frame=cvQueryFrame(capture);
        if(!frame) break;

        cvCvtColor(frame,tempFrame,CV_BGR2HSV);

        //cvSaveImage("in.jpg",frame);

        c1++;
        data = (uchar *)tempFrame->imageData; //hsv
        data1 = (uchar *)frame->imageData; //bgr



        for(i=0;i<w;i++) {
            for(j=0;j<h;j++) {

                H =((uchar *)(data + j*step))[i*channels + 0];
                S =((uchar *)(data + j*step))[i*channels + 1];
                V =((uchar *)(data + j*step))[i*channels + 2];

                B =((uchar *)(data1 + j*step))[i*channels + 0];
                G =((uchar *)(data1 + j*step))[i*channels + 1];
                R =((uchar *)(data1 + j*step))[i*channels + 2];



                if(

                        (H>85 && H<90 && S>40 && S<50 && V>90 && V<100) ||
                        (H>70 && H<85 && S>92 && S<100 && V>30 && V<40) ||
                       // (G>220 && R<55 && R>55) ||
                       // (G>220 && R<100 && B<100 && R>70 && B>70)
                        // ||
                       // (G>210 && (R-B)<15 && (B-R)<15) ||
                        //(G>80 && G<120 && R<40 && B<40)
                        getColorDistance(B,G,R,0,255,0) < thresh
                ) {
                    ((uchar *)(data1 + j*step))[i*channels + 0] = 0; //set sat to 0
                    ((uchar *)(data1 + j*step))[i*channels + 1] = 0; //set sat to 0
                    ((uchar *)(data1 + j*step))[i*channels + 2] = 0; //set val to 0
                }




            }
        }


        //cvCvtColor(tempFrame,frame,CV_HSV2BGR);
        //cvNamedWindow("frame");
        //cvShowImage("frame",frame);

        cvWriteFrame(writer,frame);
        //cvSaveImage("out.jpg",frame);
        //if(c1==2) break;
    }
    cvReleaseVideoWriter(&writer);
    qDebug("VIDEO SAVED: %d frames",c1);
    return 0;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}




