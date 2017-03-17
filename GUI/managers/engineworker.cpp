#include "engineworker.h"

EngineWorker::EngineWorker(QObject *parent) : QObject(parent){
}

void EngineWorker::booleanOperations(HeightfieldsList& he, SimpleEigenMesh& bc, BoxList& solutions) {
    Engine::booleanOperations(he, bc, solutions);
    emit finished();
}
