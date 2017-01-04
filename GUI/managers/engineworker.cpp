#include "engineworker.h"

EngineWorker::EngineWorker(QObject *parent) : QObject(parent){
}

void EngineWorker::booleanOperations(HeightfieldsList& he, IGLInterface::SimpleIGLMesh& bc, BoxList& solutions) {
    Engine::booleanOperations(he, bc, solutions);
    emit finished();
}
