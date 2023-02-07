// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet demonstrates the use of Reduced Coordinates articulations.
// ****************************************************************************

#include <ctype.h>
#include <vector>
#include <iostream>
#include "PxArticulationJointReducedCoordinate.h"
#include "PxPhysicsAPI.h"
#include "PxQueryReport.h"
#include "PxRigidActor.h"
#include "PxRigidStatic.h"
#include "PxVisualizationParameter.h"
#include "common/PxTolerancesScale.h"
#include "cooking/PxCooking.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "extensions/PxExtensionsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"
#include "extensions/PxRigidBodyExt.h"
#include "extensions/PxSimpleFactory.h"
#include "foundation/Px.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxGeometry.h"
#include "geometry/PxGeometryHelpers.h"
#include "geometry/PxGeometryHit.h"
#include "geometry/PxGeometryQuery.h"

#include "geometry/PxGjkQuery.h"
#include "extensions/PxGjkQueryExt.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "pvd/PxPvd.h"
#include "../../include/omnipvd/PxOmniPvd.h"
#include "../../pvdruntime/include/OmniPvdFileWriteStream.h"
#include "../../pvdruntime/include/OmniPvdWriter.h"

#include <memory>

using namespace physx;

static PxDefaultAllocator            gAllocator;
static PxDefaultErrorCallback          gErrorCallback;
static PxFoundation*              gFoundation    = NULL;
static PxPhysics*                gPhysics    = NULL;
static PxDefaultCpuDispatcher*          gDispatcher    = NULL;
static PxScene*                  gScene      = NULL;
static PxMaterial*                gMaterial    = NULL;
static PxOmniPvd* omniPvd = NULL;
static PxArticulationReducedCoordinate*      gArticulation  = NULL;

// TODO: Make these convex hulls (represented as meshes)
static PxRigidStatic *convexBody;
static PxRigidStatic *boxBody;

static PxConvexMeshGeometry* convexGeom;
static PxBoxGeometry* boxGeom;

void addBoxBody() {
    PxTransform pose = PxTransform(PxIdentity);
    pose.p.y += 1.0f; // Move it up
    boxBody = gPhysics->createRigidStatic(pose);

    boxGeom = new PxBoxGeometry(PxVec3(0.5f, 0.5f, 0.5f));
    PxRigidActorExt::createExclusiveShape(*boxBody, *boxGeom, *gMaterial);
    gScene->addActor(*boxBody);
}

// Uses global vars...It's a test okay
void addConvexBody() {
    // TODO: Create SSCH from 2 cuboids (extract vertices and stuff)
    PxVec3 convexVerts[] = {PxVec3(0,1,0),PxVec3(1,0,0),
                            PxVec3(-1,0,0),PxVec3(0,0,1),PxVec3(0,0,-1)};
    PxTolerancesScale tolerances;
    PxCookingParams params(tolerances); // TODO: Tolerances for what?
    PxConvexMeshDesc convexDesc;

    params.convexMeshCookingType = PxConvexMeshCookingType::eQUICKHULL;
    // disable mesh cleaning - perform mesh validation on development configurations
    params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;
    // disable edge precompute, edges are set for each triangle, slows contact generation
    params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;

    convexDesc.points.count = 5;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = convexVerts;
    // Needed when only providing vertices
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
    convexDesc.flags |= PxConvexFlag::eDISABLE_MESH_VALIDATION;
    convexDesc.flags |= PxConvexFlag::eFAST_INERTIA_COMPUTATION;

    PxConvexMesh* aConvexMesh = PxCreateConvexMesh(params, convexDesc,
                                                   gPhysics->getPhysicsInsertionCallback());

    convexGeom = new PxConvexMeshGeometry(aConvexMesh);

    PxTransform convexPose = PxTransform(PxIdentity);
       
    convexBody = gPhysics->createRigidStatic(convexPose);

    PxShape* convexShape = PxRigidActorExt::createExclusiveShape(*convexBody, *convexGeom, *gMaterial);

    convexShape->setRestOffset(1.0f);

    // Set rest offset
    gScene->addActor(*convexBody);
}

void initPhysics(bool /*interactive*/)
{

    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    omniPvd = PxCreateOmniPvd(*gFoundation);
    OmniPvdWriter* omniWriter = omniPvd->getWriter();
    // omniWriter->setLogFunction([](char *logMsg) {std::cout << logMsg << std::endl;});

    OmniPvdFileWriteStream* omniFileWriteStream = omniPvd->getFileWriteStream();
    omniWriter->setWriteStream(omniFileWriteStream);

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, nullptr, omniPvd);

    omniFileWriteStream->setFileName("/tmp/myoutpufile.ovd");
    omniPvd->startSampling();

    PxInitExtensions(*gPhysics, nullptr);

    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

    PxU32 numCores = SnippetUtils::getNbPhysicalCores();
    gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
    sceneDesc.cpuDispatcher  = gDispatcher;
    sceneDesc.filterShader  = PxDefaultSimulationFilterShader;

    sceneDesc.solverType = PxSolverType::eTGS;

    gScene = gPhysics->createScene(sceneDesc);
    gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f); // TODO: Needed?

    gMaterial = gPhysics->createMaterial(0.5f, 0.8f, 0.3f);

    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);

    // Add actors
    gScene->addActor(*groundPlane);
    // Add collision body actors (SSCH and cuboid)
    addConvexBody();
    addBoxBody();

    gScene->setGravity(PxVec3(0,0,0));
    gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1);
    gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 1);
}

// static bool gClosing = true;
static int tick = 0;
void stepPhysics(bool /*interactive*/)
{
    int tick_every = 500;
    tick++;
    if(tick > tick_every)
    {
        tick= 0;
        const PxReal dt = 1.0f / 60.f;

        gScene->simulate(dt);
        gScene->fetchResults(true);

    }

}

void cleanupPhysics(bool /*interactive*/)
{
    gArticulation->release();
    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
    PX_RELEASE(gPhysics);
    PX_RELEASE(omniPvd);
    PxCloseExtensions();  
    PX_RELEASE(gFoundation);

    printf("SnippetArticulation done.\n");
}


void renderCallback()
{
    stepPhysics(true);

    const PxVec3 dynColor(1.0f, 0.5f, 0.25f);
    const PxVec3 rcaColor(0.6f*0.75f, 0.8f*0.75f, 1.0f*0.75f);

    PxScene* scene;
    PxGetPhysics().getScenes(&scene,1);
    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    if(nbActors)
    {
        std::vector<PxRigidActor*> actors(nbActors);
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
        Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
    }

    // Render witness points using simple OpenGL line
    auto convexSupport = PxGjkQueryExt::ConvexMeshSupport(*convexGeom);
    auto boxSupport = PxGjkQueryExt::BoxSupport(*boxGeom);

    PxVec3 pointA;
    PxVec3 pointB;
    PxVec3 separatingAxis;
    PxReal separation;

    PxGjkQuery::proximityInfo(convexSupport, boxSupport, convexBody->getGlobalPose(), boxBody->getGlobalPose(), 0, 0, pointA, pointB, separatingAxis, separation);

    // Draw line between witness points (only visible after pressing
    // 'n' in the snippet OpenGL visualizer)
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(pointA.x, pointA.y, pointA.z);
    glVertex3f(pointB.x, pointB.y, pointB.z);
    glEnd();

    PxGeomRaycastHit rayHits;
    PxHitFlags flags = PxHitFlag::ePOSITION|PxHitFlag::eNORMAL|PxHitFlag::eUV;
    PxRaycastBuffer buffer;

    // "BACKS UP" THE RAY BY 0.1. IF THE LINE BELOW IS COMMENTED OUT,
    // `raycast` WILL STILL RETURN HIT DATA, BUT FACE INDEX WILL BE
    // NULL!
    pointA.y += 0.1;

    auto numHits = PxGeometryQuery::raycast(pointA, separatingAxis, *convexGeom, convexBody->getGlobalPose(), 1.0f, flags, 2, &rayHits, sizeof(PxGeomRaycastHit));
    if (numHits == 0) {
        printf("RAYCAST FAILED!\n");
    } else {
        printf("Face Idx: %d\n", rayHits.faceIndex);
    }
    PxHullPolygon hitPolygon;
    convexGeom->convexMesh->getPolygonData(rayHits.faceIndex, hitPolygon);

    Snippets::finishRender();
}

void exitCallback(void)
{
    cleanupPhysics(true);
}

const PxVec3 gCamEyeLift(-5.858525f, 1.0f, 1.546743f); // y=6.079476f
const PxVec3 gCamDirLift(0.927923f, -0.356565f, -0.108720f);

int snippetMain(int, const char*const*)
{
    extern void renderLoop();
    renderLoop();
    return 0;
}
