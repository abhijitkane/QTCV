#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


double getDist(int x1,int y1, int x2, int y2) {
    return ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2));
}

int overlap(CvPoint pt0,CvPoint pt1,CvPoint pt2,CvPoint pt3,int x, int y) {
    int xl=0,xm=0,yl=0,ym=0;
    if(pt0.x>=x) xl++;
    if(pt0.y>=y) yl++;
    if(pt0.x<x) xm++;
    if(pt0.y<y) ym++;

    if(pt1.x>=x) xl++;
    if(pt1.y>=y) yl++;
    if(pt1.x<x) xm++;
    if(pt1.y<y) ym++;

    if(pt2.x>=x) xl++;
    if(pt2.y>=y) yl++;
    if(pt2.x<x) xm++;
    if(pt2.y<y) ym++;

    if(pt3.x>=x) xl++;
    if(pt3.y>=y) yl++;
    if(pt3.x<x) xm++;
    if(pt3.y<y) ym++;

    if(xm>0 && xl>0 && yl>0 && ym>0 ) return 1;
    return 0;

}


CvSeq* findSquares4( IplImage* img, CvMemStorage* storage )
{
    int CHANNELS=3;
    CvSeq* contours;
    int i, c, l, N = 11;
    int thresh=50;
    CvSize sz = cvSize( img->width & -2, img->height & -2 );
    IplImage* timg = cvCloneImage( img ); // make a copy of input image
    IplImage* gray = cvCreateImage( sz, 8, 1 );
    IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, CHANNELS );
    IplImage* tgray;
    CvSeq* result;
    double s, t;
    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));

    // down-scale and upscale the image to filter out the noise
    cvPyrDown( timg, pyr, 7 );
    cvPyrUp( pyr, timg, 7 );
    tgray = cvCreateImage( sz, 8, 1 );

    // find squares in every color plane of the image
    for( c = 0; c < CHANNELS; c++ )
    {
        // extract the c-th color plane
        cvSetImageCOI( timg, c+1 );
        cvCopy( timg, tgray, 0 );

        // try several threshold levels
        for( l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cvCanny( tgray, gray, 0, thresh, 5 );
                // dilate canny output to remove potential
                // holes between edge segments
                cvDilate( gray, gray, 0, 1 );
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );
            }

            // find contours and store them all as a list
            cvFindContours( gray, storage, &contours, sizeof(CvContour),
                CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

            // test each contour
            while( contours )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                result = cvApproxPoly( contours, sizeof(CvContour), storage,
                    CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( result->total == 4 &&
                    fabs(cvContourArea(result,CV_WHOLE_SEQ)) > 1000 &&
                    cvCheckContourConvexity(result) )
                {
                    s = 0;

                    for( i = 0; i < 5; i++ )
                    {
                        // find minimum angle between joint
                        // edges (maximum of cosine)
                        if( i >= 2 )
                        {
                            t = fabs(angle(
                            (CvPoint*)cvGetSeqElem( result, i ),
                            (CvPoint*)cvGetSeqElem( result, i-2 ),
                            (CvPoint*)cvGetSeqElem( result, i-1 )));
                            s = s > t ? s : t;
                        }
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( s < 0.15 )
                        for( i = 0; i < 4; i++ )
                            cvSeqPush( squares,
                                (CvPoint*)cvGetSeqElem( result, i ));
                }

                // take the next contour
                contours = contours->h_next;
            }
        }
    }

    // release all the temporary images
    cvReleaseImage( &gray );
    cvReleaseImage( &pyr );
    cvReleaseImage( &tgray );
    cvReleaseImage( &timg );

    return squares;
}


CvPoint *drawSquares( IplImage* img, CvSeq* squares)
{
   CvSeqReader reader;

    //IplImage* cpy = cvCreateImage( cvSize(img->width, img->height), 8, 3 );
    //cvZero(cpy);
    int i,j;
    int temp=0;
    CvScalar s=cvScalar(0);
    int txs,tys;
    int t2;
    int oldx[4],oldy[4];
    static CvPoint sqFinal[5];

    for(i=0;i<5;i++) sqFinal[i]=cvPoint(0,0);

    double dist[3];

    int oldi=0;
    // initialize reader of the sequence
    cvStartReadSeq( squares, &reader, 0 );

    // read 4 sequence elements at a time (all vertices of a square)

    for( i = 0; i < squares->total; i += 4 )
    {
        CvPoint pt[4], *rect = pt;
        int count = 4;

        // read 4 vertices
        CV_READ_SEQ_ELEM( pt[0], reader );
        CV_READ_SEQ_ELEM( pt[1], reader );
        CV_READ_SEQ_ELEM( pt[2], reader );
        CV_READ_SEQ_ELEM( pt[3], reader );

        //CHECK if any of oldx,oldy are in (pt0,1,2,3)
        //yes- > continue

        for(j=0;j<oldi;j++) {
            t2=overlap(pt[0],pt[1],pt[2],pt[3],oldx[j],oldy[j]);
            if(t2 == 1)  {
                //qDebug("Overlap: %d",t2);
                goto contLoop;
            }
        }

        txs=(pt[0].x+pt[2].x +pt[1].x + pt[3].x)/4;
        tys=(pt[0].y+pt[2].y +pt[1].y + pt[3].y)/4;


        cvCircle(img,cvPoint(txs,tys),5,cvScalar(255,0,0));



        s=cvGet2D(img,tys,txs);

        if(s.val[0]<50 && s.val[1]<50 && s.val[2]<50) {

            cvPolyLine( img, &rect, &count, 1, 1, cvScalar(0,0,255) , 3, CV_AA, 0 );
            oldx[oldi]=txs;
            oldy[oldi++]=tys;

        }

        contLoop: ;
    }

    //for(j=0;j<oldi;j++) qDebug("%d,%d",oldx[j],oldy[j]);

    if(oldi==4) {
        int h;
        int minI,maxI,firstI;
        for(h=1;h<4;h++) {
            dist[h-1]=getDist(oldx[0],oldy[0],oldx[h],oldy[h]);
        }
        minI=0;maxI=0;
        for(h=2;h<4;h++) {
            if(dist[h-1]>dist[maxI]) maxI=h-1;
            if(dist[h-1]<dist[minI]) minI=h-1;
        }

        maxI++;
        //maxI should be in pos2
        minI++;
        //minI should be in pos3
        firstI=(6-(maxI+minI));

        sqFinal[0]=cvPoint(oldx[0],oldy[0]);
        sqFinal[1]=cvPoint(oldx[firstI],oldy[firstI]);
        sqFinal[2]=cvPoint(oldx[maxI],oldy[maxI]);
        sqFinal[3]=cvPoint(oldx[minI],oldy[minI]);

        sqFinal[4]=cvPoint(sqrt(dist[firstI-1]),sqrt(dist[minI-1]));

    }
    else {
        sqFinal[0]=sqFinal[1]=sqFinal[2]=sqFinal[3]=sqFinal[4]=cvPoint(0,0);
    }

    //qDebug("sqFinal: %d,%d   %d,%d   %d,%d   %d,%d ",sqFinal[0].x,sqFinal[0].y,sqFinal[1].x,sqFinal[1].y,sqFinal[2].x,sqFinal[2].y,sqFinal[3].x,sqFinal[3].y);
    return sqFinal;
    // show the resultant image
    //cvShowImage( "squares1", cpy );
    //cvReleaseImage( &cpy );
}

