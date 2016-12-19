#ifndef ENGINEWORKER_H
#define ENGINEWORKER_H

#include <QObject>

#include "engine/heightfieldslist.h"
#include "engine/boxlist.h"
#include "engine/engine.h"

class EngineWorker : public QObject
{
        Q_OBJECT
    public:
        explicit EngineWorker(QObject *parent = 0);

    signals:
        void finished();
        void error(QString err);

    public slots:
        void booleanOperations(HeightfieldsList &he, IGLInterface::SimpleIGLMesh &bc, BoxList &solutions, const Dcel& inputMesh);
};

#endif // ENGINEWORKER_H
