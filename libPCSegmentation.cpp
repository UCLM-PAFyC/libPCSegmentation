// Author: David Hernandez Lopez, david.hernandez@uclm.es

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/imgproc.hpp>

#include <QFile>
#include <QTextStream>
#include <QFileInfo>
#include <QDir>

#include "LineDetection3D.h"
#include "nanoflann.hpp"
#include "utils.h"
#include "Timer.h"

#include "libPCSegmentation.h"

libPCSegmentation * libPCSegmentation::mInstance = 0;

bool libPCSegmentation::groundSegmentationByProgressiveMorphologicalFilter(QMap<QString, QVector<double> > &points,
                                                                           QVector<QString> &groundPointsIds,
                                                                           double maxWindowSize,
                                                                           double slope,
                                                                           double initialDistance,
                                                                           double maxDistance,
                                                                           QVector<QString> &toIgnorePointsIds,
                                                                           QString &strError)
{
    groundPointsIds.clear();
    double minFc=10000000.0;
    double minSc=10000000.0;
    QMap<QString,QVector<double> >::const_iterator iterPoints=points.begin();
    while(iterPoints!=points.end())
    {
        if(toIgnorePointsIds.contains(iterPoints.key()))
        {
            iterPoints++;
            continue;
        }
        float fc=iterPoints.value()[0];
        float sc=iterPoints.value()[1];
        if(fc<minFc)
        {
            minFc=fc;
        }
        if(sc<minSc)
        {
            minSc=sc;
        }
        iterPoints++;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);
    iterPoints=points.begin();
    QVector<QString> pointsIds;
    while(iterPoints!=points.end())
    {
        if(toIgnorePointsIds.contains(iterPoints.key()))
        {
            iterPoints++;
            continue;
        }
        double fc=iterPoints.value()[0]-minFc;
        double sc=iterPoints.value()[1]-minSc;
        double tc=iterPoints.value()[2];
        cloud->push_back(pcl::PointXYZ(fc,sc,tc));
        pointsIds.push_back(iterPoints.key());
        iterPoints++;
    }
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud (cloud);
    pmf.setMaxWindowSize (maxWindowSize);
    pmf.setSlope (slope);
    pmf.setInitialDistance (initialDistance);
    pmf.setMaxDistance (maxDistance);
    pmf.extract (ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (ground);
    extract.filter (*cloud_filtered);
    for (size_t i = 0; i < ground->indices.size (); ++i)
    {
        int pointPos = ground->indices[i];
        QString pointId=pointsIds[pointPos];
        groundPointsIds.push_back(pointId);
    }
    return(true);
}

bool libPCSegmentation::minimumAreaRotatedRectangle(QVector<QVector<double> > &points,
                                                    QVector<QVector<double> > &rectanglePoints,
                                                    QString &strError)
{
    rectanglePoints.clear();
    double meanX=0.0;
    double meanY=0.0;
    for( int i = 0; i < points.size(); i++ )
    {
        meanX+=points[i][0];
        meanY+=points[i][1];
    }
    meanX=meanX/(double(points.size()));
    meanY=meanY/(double(points.size()));
    vector<cv::Point2f> cvPoints;
    // Generate a random set of points
    for( int i = 0; i < points.size(); i++ )
    {
        cv::Point2f pt;
        pt.x = points[i][0]-meanX;
        pt.y = points[i][1]-meanY;
        cvPoints.push_back(pt);
    }
    // Find the minimum area enclosing bounding box
    cv::Point2f vtx[4];
    cv::RotatedRect box = cv::minAreaRect(cvPoints);
    box.points(vtx);
    rectanglePoints.resize(4);
    for(int i=0;i<4;i++)
    {
        cv::Point2f pt=vtx[i];
        QVector<double> corner(2);
        corner[0]=pt.x+meanX;
        corner[1]=pt.y+meanY;
        rectanglePoints[i]=corner;
    }
    return(true);
}

bool libPCSegmentation::threeDLineDeteccion(QVector<QVector<double> >& points,
                                            QString outputPath,
                                            QVector<QVector<QVector<QVector<QVector<double> > > > >& linesByPlane,
                                            QString& strError)
{
    linesByPlane.clear();
    QDir currentDir=QDir::currentPath();
    if(!currentDir.exists(outputPath))
    {
        if(!currentDir.mkpath(outputPath))
        {
            strError=QObject::tr("libPCSegmentation::threeDLineDeteccion");
            strError+=QObject::tr("\nError making path: %1").arg(outputPath);
            return(false);
        }
    }
    PointCloud<double> pointData;
    for(int np=0;np<points.size();np++)
    {
        double x=points[np][0];
        double y=points[np][1];
        double z=points[np][2];
        pointData.pts.push_back(PointCloud<double>::PtData(x,y,z));
    }
    int k = 20;
    LineDetection3D detector;
    std::vector<PLANE> planes;
    std::vector<std::vector<cv::Point3d> > lines;
    std::vector<double> ts;
    detector.run( pointData, k, planes, lines, ts );
//    cout<<"lines number: "<<lines.size()<<endl;
//    cout<<"planes number: "<<planes.size()<<endl;
//        writeOutPlanes( fileOut, planes, detector.scale );
    QString planesOutputFileName=outputPath+"/"+LIBPCSEGMENTATION_PLANES_TXT_FILE_NAME;
    QFile file(planesOutputFileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("libPCSegmentation::threeDLineDeteccion");
        strError+=QObject::tr("\nError opening file: %1").arg(planesOutputFileName);
        return(false);
    }
    QTextStream strOut(&file);
    linesByPlane.resize(planes.size());
    for (int p=0; p<planes.size(); ++p)
    {
        QString planeFileName=outputPath+"/"+LIBPCSEGMENTATION_PLANE_PREFIX_TXT_FILE_NAME;
        planeFileName+=(QString::number(p)+".txt");
        QFile planeFile(planeFileName);
        if(!planeFile.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            strError=QObject::tr("libPCSegmentation::threeDLineDeteccion");
            strError+=QObject::tr("\nError opening file: %1").arg(planeFileName);
            return(false);
        }
        QTextStream planeStrOut(&planeFile);
        QString planesFileNameForRead=outputPath+"/"+LIBPCSEGMENTATION_PLANE_PREFIX_TXT_FILE_NAME_FOR_READ;
        planesFileNameForRead+=(QString::number(p)+".txt");
        QFile fileForRead(planesFileNameForRead);
        if(!fileForRead.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            strError=QObject::tr("libPCSegmentation::threeDLineDeteccion");
            strError+=QObject::tr("\nError opening file: %1").arg(planesFileNameForRead);
            return(false);
        }
        QTextStream strOutForRead(&fileForRead);
        int R = rand()%255;
        int G = rand()%255;
        int B = rand()%255;
        QVector<QVector<QVector<QVector<double> > > > planeLines(planes[p].lines3d.size());
        strOutForRead<<QString::number(planes[p].lines3d.size())<<"\n";
        for (int i=0; i<planes[p].lines3d.size(); ++i)
        {
            QVector<QVector<QVector<double> > > planeLine(planes[p].lines3d[i].size());
            strOutForRead<<QString::number(planes[p].lines3d[i].size())<<"\n";
            for (int j=0; j<planes[p].lines3d[i].size(); ++j)
            {
                cv::Point3d dev = planes[p].lines3d[i][j][1] - planes[p].lines3d[i][j][0];
                double L = sqrt(dev.x*dev.x + dev.y*dev.y + dev.z*dev.z);
                int kk = L/(detector.scale/10);
                double x = planes[p].lines3d[i][j][0].x, y = planes[p].lines3d[i][j][0].y, z = planes[p].lines3d[i][j][0].z;
                double dx = dev.x/kk, dy = dev.y/kk, dz = dev.z/kk;
                QVector<QVector<double> > points(kk);
                strOutForRead<<QString::number(kk)<<"\n";
                for ( int jj=0; jj<kk; ++jj)
                {
                    x += dx;
                    y += dy;
                    z += dz;
                    QVector<double> point(3);
                    point[0]=x;
                    point[1]=y;
                    point[2]=z;
                    points[jj]=point;
                    strOut<<QString::number(x,'f',4)<<" ";
                    strOut<<QString::number(y,'f',4)<<" ";
                    strOut<<QString::number(z,'f',4)<<" ";
                    strOut<<QString::number(R)<<" ";
                    strOut<<QString::number(G)<<" ";
                    strOut<<QString::number(B)<<" ";
                    strOut<<QString::number(p)<<" ";
                    strOut<<"\n";
                    strOutForRead<<QString::number(x,'f',4)<<" ";
                    strOutForRead<<QString::number(y,'f',4)<<" ";
                    strOutForRead<<QString::number(z,'f',4)<<"\n";
                    planeStrOut<<QString::number(x,'f',4)<<" ";
                    planeStrOut<<QString::number(y,'f',4)<<" ";
                    planeStrOut<<QString::number(z,'f',4)<<" ";
                    planeStrOut<<QString::number(R)<<" ";
                    planeStrOut<<QString::number(G)<<" ";
                    planeStrOut<<QString::number(B)<<" ";
                    planeStrOut<<QString::number(p)<<" ";
                    planeStrOut<<"\n";
                }
                planeLine[j]=points;
            }
            planeLines[i]=planeLine;
        }
        linesByPlane[p]=planeLines;
        planeFile.close();
        fileForRead.close();
    }
    file.close();
    //        writeOutLines( fileOut, lines, detector.scale );
    QString linesFileName=outputPath+"/"+LIBPCSEGMENTATION_LINES_TXT_FILE_NAME;
    QFile linesFile(linesFileName);
    if(!linesFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        strError=QObject::tr("libPCSegmentation::threeDLineDeteccion");
        strError+=QObject::tr("\nError opening file: %1").arg(linesFileName);
        return(false);
    }
    QTextStream linesStrOut(&linesFile);
    for (int p=0; p<lines.size(); ++p)
    {
        int R = rand()%255;
        int G = rand()%255;
        int B = rand()%255;

        cv::Point3d dev = lines[p][1] - lines[p][0];
        double L = sqrt(dev.x*dev.x + dev.y*dev.y + dev.z*dev.z);
        int k = L/(detector.scale/10);

        double x = lines[p][0].x, y = lines[p][0].y, z = lines[p][0].z;
        double dx = dev.x/k, dy = dev.y/k, dz = dev.z/k;
        for ( int j=0; j<k; ++j)
        {
            x += dx;
            y += dy;
            z += dz;

            linesStrOut<<QString::number(x,'f',4)<<" ";
            linesStrOut<<QString::number(y,'f',4)<<" ";
            linesStrOut<<QString::number(z,'f',4)<<" ";
            linesStrOut<<QString::number(R)<<" ";
            linesStrOut<<QString::number(G)<<" ";
            linesStrOut<<QString::number(B)<<" ";
            linesStrOut<<QString::number(p)<<" ";
            linesStrOut<<"\n";

        }
    }
    linesFile.close();
    return(true);
}

bool libPCSegmentation::removeOutliers(QMap<QString, QVector<double> > &points,
                                       QVector<QString> &outliersPointsIds,
                                       bool algorithmRemoveOutliersRadius,
                                       bool algorithmRemoveOutliersStatistical,
                                       int statisticalSampleNeighbors,
                                       double statisticalStdThreshold,
                                       double radiusSearch,
                                       int radiusMinimumNeighbors,
                                       QString &strError)
{
    outliersPointsIds.clear();
    if(!algorithmRemoveOutliersRadius&&!algorithmRemoveOutliersStatistical)
    {
        return(true);
    }
    double minFc=10000000.0;
    double minSc=10000000.0;
    QMap<QString,QVector<double> >::const_iterator iterPoints=points.begin();
    while(iterPoints!=points.end())
    {
        float fc=iterPoints.value()[0];
        float sc=iterPoints.value()[1];
        if(fc<minFc)
        {
            minFc=fc;
        }
        if(sc<minSc)
        {
            minSc=sc;
        }
        iterPoints++;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr firstInputCloud (new pcl::PointCloud<pcl::PointXYZ>);
    iterPoints=points.begin();
    QVector<QString> firstPointsIds;
    while(iterPoints!=points.end())
    {
        double fc=iterPoints.value()[0]-minFc;
        double sc=iterPoints.value()[1]-minSc;
        double tc=iterPoints.value()[2];
        firstInputCloud->push_back(pcl::PointXYZ(fc,sc,tc));
        firstPointsIds.push_back(iterPoints.key());
        iterPoints++;
    }
    if(algorithmRemoveOutliersRadius)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(firstInputCloud);
        outrem.setRadiusSearch(radiusSearch);
        outrem.setMinNeighborsInRadius(radiusMinimumNeighbors);
        outrem.setNegative (true); // para que queden los eliminados
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_rem (new pcl::PointCloud<pcl::PointXYZ>);
        outrem.filter (*cloud_filtered_rem);
        for(size_t i=0;i<cloud_filtered_rem->size();i++)
        {
            float rpFc=cloud_filtered_rem->points[i].x;
            float rpSc=cloud_filtered_rem->points[i].y;
            float rpTc=cloud_filtered_rem->points[i].z;
            iterPoints=points.begin();
            while(iterPoints!=points.end())
            {
                QString pointId=iterPoints.key();
                if(outliersPointsIds.contains(pointId))
                {
                    iterPoints++;
                    continue;
                }
                double fc=iterPoints.value()[0]-minFc;
                double sc=iterPoints.value()[1]-minSc;
                double tc=iterPoints.value()[2];
                double distance=sqrt(pow(rpFc-fc,2.0)+pow(rpSc-sc,2.0)+pow(rpTc-tc,2.0));
                if(distance<0.00001)
                {
                    outliersPointsIds.push_back(pointId);
                    break;
                }
                iterPoints++;
            }
        }
    }

    if(algorithmRemoveOutliersStatistical)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr secondInputCloud (new pcl::PointCloud<pcl::PointXYZ>);
        iterPoints=points.begin();
        QVector<QString> secondPointsIds;
        while(iterPoints!=points.end())
        {
            if(outliersPointsIds.contains(iterPoints.key()))
            {
                iterPoints++;
                continue;
            }
            double fc=iterPoints.value()[0]-minFc;
            double sc=iterPoints.value()[1]-minSc;
            double tc=iterPoints.value()[2];
            secondInputCloud->push_back(pcl::PointXYZ(fc,sc,tc));
            secondPointsIds.push_back(iterPoints.key());
            iterPoints++;
        }
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outsor;
        outsor.setInputCloud (secondInputCloud);
        outsor.setMeanK (statisticalSampleNeighbors);
        outsor.setStddevMulThresh(statisticalStdThreshold);
        outsor.setNegative (true); // para que queden los eliminados
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor (new pcl::PointCloud<pcl::PointXYZ>);
        outsor.filter (*cloud_filtered_sor);
        for(size_t i=0;i<cloud_filtered_sor->size();i++)
        {
            float rpFc=cloud_filtered_sor->points[i].x;
            float rpSc=cloud_filtered_sor->points[i].y;
            float rpTc=cloud_filtered_sor->points[i].z;
            iterPoints=points.begin();
            while(iterPoints!=points.end())
            {
                QString pointId=iterPoints.key();
                if(outliersPointsIds.contains(pointId))
                {
                    iterPoints++;
                    continue;
                }
                double fc=iterPoints.value()[0]-minFc;
                double sc=iterPoints.value()[1]-minSc;
                double tc=iterPoints.value()[2];
                double distance=sqrt(pow(rpFc-fc,2.0)+pow(rpSc-sc,2.0)+pow(rpTc-tc,2.0));
                if(distance<0.00001)
                {
                    outliersPointsIds.push_back(pointId);
                    break;
                }
                iterPoints++;
            }
        }
    }

    return(true);
}

bool libPCSegmentation::spatialClusterExtraction(QVector<QVector<double> > &points,
                                                 double clusterTolerance,
                                                 int minClusterSize,
                                                 QVector<QVector<int> > &clustersPointsIndex,
                                                 QString &strError,
                                                 bool useZ)
{
    strError.clear();
    clustersPointsIndex.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int np=0;np<points.size();np++)
    {
        float fc=points[np][0];
        float sc=points[np][1];
        float tc=0.0;
        if(useZ) tc=points[np][2];
        cloud->push_back(pcl::PointXYZ(fc,sc,tc));
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (10000000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract(cluster_indices);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        QVector<int> clusterPointsIndex;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            clusterPointsIndex.push_back(*pit);
//            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        }
        clustersPointsIndex.push_back(clusterPointsIndex);
    }
    return(true);
}
