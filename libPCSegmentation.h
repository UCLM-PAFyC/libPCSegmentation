// Author: David Hernandez Lopez, david.hernandez@uclm.es
#ifndef LIBPCSEGMENTATION_H

#define LIBPCSEGMENTATION_H

#define LIBPCSEGMENTATION_PLANES_TXT_FILE_NAME          "Planes.txt"
#define LIBPCSEGMENTATION_LINES_TXT_FILE_NAME          "Lines.txt"
#define LIBPCSEGMENTATION_PLANE_PREFIX_TXT_FILE_NAME          "Plane_"
#define LIBPCSEGMENTATION_PLANE_PREFIX_TXT_FILE_NAME_FOR_READ          "Plane_ForRead_"

#include <QMap>
#include <QVector>
#include <QString>

#include "libPCSegmentation_global.h"

class LIBPCSEGMENTATIONSHARED_EXPORT libPCSegmentation
{

public:
    inline libPCSegmentation() {;};
    static inline libPCSegmentation * getInstance(void )
    {
        if ( mInstance == 0 ) mInstance = new libPCSegmentation;
            return mInstance;
    };

    // http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php
    bool groundSegmentationByProgressiveMorphologicalFilter(QMap<QString,QVector<double> >& points,
                                                            QVector<QString> &groundPointsIds,
                                                            double maxWindowSize,
                                                            double slope,
                                                            double initialDistance,
                                                            double maxDistance,
                                                            QVector<QString> &toIgnorePointsIds,
                                                            QString& strError);
    bool minimumAreaRotatedRectangle(QVector<QVector<double> >& points,
                                     QVector<QVector<double> >& rectanglePoints,
                                     QString& strError);
    bool threeDLineDeteccion(QVector<QVector<double> >& points,
                             QString outputPath,
                             QVector<QVector<QVector<QVector<QVector<double> > > > > &linesByPlane,
                             QString& strError);

    // http://pointclouds.org/documentation/tutorials/remove_outliers.php
    bool removeOutliers(QMap<QString,QVector<double> >& points,
                        QVector<QString> &outliersPointsIds,
                        bool algorithmRemoveOutliersRadius,
                        bool algorithmRemoveOutliersStatistical,
                        int statisticalSampleNeighbors,
                        double statisticalStdThreshold,
                        double radiusSearch,
                        int radiusMinimumNeighbors,
                        QString& strError);
    bool spatialClusterExtraction(QVector<QVector<double> > &points,
                                  double clusterTolerance,
                                  int minClusterSize,
                                  QVector<QVector<int> > &clustersPointsIndex,
                                  QString &strError,
                                  bool useZ=true);
private:
    static libPCSegmentation * mInstance;
};

#endif // LIBPCSEGMENTATION_H
