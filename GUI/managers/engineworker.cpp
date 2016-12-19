#include "engineworker.h"

EngineWorker::EngineWorker(QObject *parent) : QObject(parent){
}

void EngineWorker::booleanOperations(HeightfieldsList& he, IGLInterface::SimpleIGLMesh& bc, BoxList& solutions, const Dcel& inputMesh) {
    Engine::booleanOperations(he, bc, solutions, inputMesh);
    emit finished();
}
