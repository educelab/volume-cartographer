//
// Created by Seth Parker on 3/14/16.
//

#include <BulletSoftBody/btDefaultSoftBodySolver.h>

#include "vc/meshing/DeepCopy.hpp"
#include "vc/meshing/ITK2Bullet.hpp"
#include "vc/experimental/texturing/ClothModelingUVMapping.hpp"

//// Callbacks ////
/* Note: Callbacks won't compile if they're not wrapped in the namespace.
 * `using namespace volcart::texturing` doesn't work. I'm assuming this is some
 * terrible Bullet Physics thing. - SP, 08-01-2016 */

namespace volcart
{
namespace experimental {
    namespace texturing {
// Forward pretick callbacks to functions in a stupid Bullet physics way
        static void
        constrainMotionCallback(btDynamicsWorld *world, btScalar timeStep) {
            auto w = static_cast<ClothModelingUVMapping *>(world->getWorldUserInfo());
            w->cbConstrainMotion(timeStep);
        }

        static void
        axisLockCallback(btDynamicsWorld *world, btScalar timeStep) {
            auto w = static_cast<ClothModelingUVMapping *>(world->getWorldUserInfo());
            w->cbAxisLock(timeStep);
        }

        static void
        moveTowardTargetCallback(btDynamicsWorld *world, btScalar timeStep) {
            auto w = static_cast<ClothModelingUVMapping *>(world->getWorldUserInfo());
            w->cbMoveTowardTarget(timeStep);
        }

        static void
        emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
            auto w = static_cast<ClothModelingUVMapping *>(world->getWorldUserInfo());
            w->cbEmptyPreTick(timeStep);
        }


    }  // texturing
} // experimental
}  // volcart

using namespace volcart::experimental::texturing;

// Constructor
ClothModelingUVMapping::ClothModelingUVMapping(
    volcart::ITKMesh::Pointer input,
    uint16_t unfurlIterations,
    uint16_t collideIterations,
    uint16_t expandIterations,
    VertIDList unfurlPins,
    VertIDList expansionPins)
    : mesh_(input)
    , unfurlIterations_(unfurlIterations)
    , unfurlA_(10)
    , unfurlPins_(std::move(unfurlPins))
    , collideIterations_(collideIterations)
    , collisionA_(-10)
    , expandIterations_(expandIterations)
    , expansionA_(-10)
    , expansionPins_(std::move(expansionPins))
{
    // Create Dynamic world for bullet cloth simulation
    worldBroadphase_ = new btDbvtBroadphase();
    worldCollisionConfig_ = new btSoftBodyRigidBodyCollisionConfiguration();
    worldCollisionDispatcher_ =
        new btCollisionDispatcher(worldCollisionConfig_);
    worldConstraintSolver_ = new btSequentialImpulseConstraintSolver();
    worldSoftBodySolver_ = new btDefaultSoftBodySolver();
    world_ = new btSoftRigidDynamicsWorld(
        worldCollisionDispatcher_, worldBroadphase_, worldConstraintSolver_,
        worldCollisionConfig_, worldSoftBodySolver_);

    // Add the collision plane at the origin
    btTransform startTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
    btScalar mass = 0.f;
    btVector3 localInertia(0, 0, 0);
    btVector3 planeNormal(0, 1, 0);
    btCollisionShape* groundShape =
        new btStaticPlaneShape(planeNormal, 0);  // Normal + offset along normal
    auto myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(
        mass, myMotionState, groundShape, localInertia);

    collisionPlane_ = new btRigidBody(cInfo);
    collisionPlane_->setUserIndex(-1);
    world_->addRigidBody(collisionPlane_);

    // Convert mesh to a softbody
    volcart::meshing::ITK2Bullet(mesh_, world_->getWorldInfo(), &softBody_);

    // Scale the mesh so that max dimension <= 80m
    // Note: Assumes max is a positive coordinate. Not sure what this will do
    // for small meshes
    btVector3 min, max;
    softBody_->getAabb(min, max);
    double maxDim = (max.getX() < max.getY()) ? max.getY() : max.getX();
    maxDim = (maxDim < max.getZ()) ? max.getZ() : maxDim;

    meshToWorldScale_ = 80 / maxDim;
    softBody_->scale(btVector3(
        static_cast<float>(meshToWorldScale_),
        static_cast<float>(meshToWorldScale_),
        static_cast<float>(meshToWorldScale_)));
    startingSurfaceArea_ = surface_area_();

    // Set the mass for the whole cloth
    for (int i = 0; i < softBody_->m_nodes.size(); ++i) {
        softBody_->setMass(i, 1);
    }

    // Add the softbody to the dynamics world
    softBody_->randomizeConstraints();
    softBody_->updateNormals();
    world_->addSoftBody(softBody_);
}

// Destructor
ClothModelingUVMapping::~ClothModelingUVMapping()
{
    world_->removeSoftBody(softBody_);
    delete softBody_;

    world_->removeRigidBody(collisionPlane_);
    delete collisionPlane_->getMotionState();
    delete collisionPlane_;

    delete world_;
    delete worldConstraintSolver_;
    delete worldSoftBodySolver_;
    delete worldCollisionConfig_;
    delete worldCollisionDispatcher_;
    delete worldBroadphase_;
}

// Process this mesh
volcart::UVMap ClothModelingUVMapping::compute()
{
    if (unfurlIterations_ > 0) {
        unfurl_();
    }
    if (collideIterations_ > 0) {
        collide_();
    }
    if (expandIterations_ > 0) {
        expand_();
    }

    return getUVMap();
}

// Call the stages individually
void ClothModelingUVMapping::unfurl() { unfurl_(); };
void ClothModelingUVMapping::collide() { collide_(); };
void ClothModelingUVMapping::expand() { expand_(); };

// Get UV Map created from flattened object
volcart::UVMap ClothModelingUVMapping::getUVMap()
{
    // Get the current XZ bounds of the softbody
    btVector3 min, max;
    softBody_->getAabb(min, max);

    // Round so that we have integer bounds
    double umin = std::floor(min.getX());
    double umax = std::ceil(max.getX());
    double vmin = std::floor(min.getZ());
    double vmax = std::ceil(max.getZ());

    // Scale width and height back to volume coordinates
    double aspectWidth = std::abs(umax - umin) * (1 / meshToWorldScale_);
    double aspectHeight = std::abs(vmax - vmin) * (1 / meshToWorldScale_);

    volcart::UVMap uvMap;
    uvMap.ratio(aspectWidth, aspectHeight);

    // Calculate uv coordinates
    double u, v;
    for (auto i = 0; i < softBody_->m_nodes.size(); ++i) {
        u = (softBody_->m_nodes[i].m_x.getX() - umin) / (umax - umin);
        v = (softBody_->m_nodes[i].m_x.getZ() - vmin) / (vmax - vmin);
        cv::Vec2d uv(u, v);

        // Add the uv coordinates into our map at the point index specified
        uvMap.set(i, uv);
    }

    return uvMap;
}

// Get mesh version of flattened object
// Note: This is still in world coordinates, not volume coordinates
volcart::ITKMesh::Pointer ClothModelingUVMapping::getMesh()
{
    auto output = volcart::ITKMesh::New();
    volcart::meshing::DeepCopy(mesh_, output);
    volcart::meshing::Bullet2ITK(softBody_, output);
    return output;
}

///// Simulation /////
// Use gravity to unfurl the cloth
void ClothModelingUVMapping::unfurl_()
{
    // Set the simulation parameters
    world_->setInternalTickCallback(
        constrainMotionCallback, static_cast<void*>(this), true);
    world_->setGravity(btVector3(static_cast<float>(unfurlA_), 0.0f, 0.0f));
    softBody_->getWorldInfo()->m_gravity = world_->getGravity();

    // Damping coefficient of the soft body [0,1]
    softBody_->m_cfg.kDP = 0.01f;
    // Linear stiffness coefficient [0,1]
    softBody_->m_materials[0]->m_kLST = 1.0;
    // Area/Angular stiffness coefficient [0,1]
    softBody_->m_materials[0]->m_kAST = 1.0;
    // Volume stiffness coefficient [0,1]
    softBody_->m_materials[0]->m_kVST = 1.0;

    // Set the pins to not move
    for (auto pin : unfurlPins_) {
        softBody_->setMass(pin, 0.f);
    }

    // Run the simulation
    for (uint16_t i = 0; i < unfurlIterations_; ++i) {
        std::cerr << "volcart::texturing::clothUV: Unfurling " << i + 1 << "/"
                  << unfurlIterations_ << "\r" << std::flush;
        world_->stepSimulation(1 / 60.f, 10);
        softBody_->solveConstraints();
    }
    std::cerr << std::endl;
}

// Collide the cloth with the plane
void ClothModelingUVMapping::collide_()
{
    // Set the simulation parameters
    world_->setInternalTickCallback(
        axisLockCallback, static_cast<void*>(this), true);
    world_->setGravity(btVector3(0.0f, static_cast<float>(collisionA_), 0.0f));
    collisionPlane_->setFriction(0);  // (0-1] Default: 0.5
    softBody_->getWorldInfo()->m_gravity = world_->getGravity();
    softBody_->m_cfg.kDF =
        0.1f;  // Dynamic friction coefficient (0-1] Default: 0.2
    softBody_->m_cfg.kDP = 0.01f;  // Damping coefficient of the soft body [0,1]

    // Reset all pins to move
    for (auto n = 0; n < softBody_->m_nodes.size(); ++n) {
        softBody_->setMass(n, 1);
    }

    // Run the simulation
    for (uint16_t i = 0; i < collideIterations_; ++i) {
        std::cerr << "volcart::texturing::clothUV: Colliding " << i + 1 << "/"
                  << collideIterations_ << "\r" << std::flush;
        world_->stepSimulation(1 / 60.f, 10);
        softBody_->solveConstraints();
    }
    std::cerr << std::endl;
}

// Expand the edges of the cloth to iron out wrinkles, then let it relax
void ClothModelingUVMapping::expand_()
{
    // Set the simulation parameters
    world_->setInternalTickCallback(
        moveTowardTargetCallback, static_cast<void*>(this), true);
    world_->setGravity(btVector3(0, 0, 0));
    collisionPlane_->setFriction(0);  // (0-1] Default: 0.5
    softBody_->getWorldInfo()->m_gravity = world_->getGravity();

    // Dynamic friction coefficient (0-1] Default: 0.2
    softBody_->m_cfg.kDF = 0.1f;
    // Damping coefficient of the soft body [0,1]
    softBody_->m_cfg.kDP = 0.01f;
    // Linear stiffness coefficient [0,1]
    softBody_->m_materials[0]->m_kLST = 1.0;
    // Area/Angular stiffness coefficient [0,1]
    softBody_->m_materials[0]->m_kAST = 1.0;
    // Volume stiffness coefficient [0,1]
    softBody_->m_materials[0]->m_kVST = 1.0;

    // Get the current XZ center of the softBody
    btVector3 min, max;
    softBody_->getAabb(min, max);
    btVector3 center;
    center.setX((max.getX() + min.getX()) / 2);
    center.setY(0);
    center.setZ((max.getZ() + min.getZ()) / 2);

    // Setup the expansion pins
    Pin newPin;
    currentPins_.clear();
    for (auto pin : expansionPins_) {
        newPin.index = pin;
        newPin.node = &softBody_->m_nodes[pin];
        newPin.node->m_x.setY(0);  // Force the pin to the Y-plane
        newPin.target = (newPin.node->m_x - center) * 1.5;

        currentPins_.push_back(newPin);
    }

    // Expand the edges
    for (uint16_t i = 0; i < expandIterations_; ++i) {
        std::cerr << "volcart::texturing::clothUV: Expanding " << i + 1 << "/"
                  << expandIterations_ << "\r" << std::flush;
        world_->stepSimulation(1 / 60.f, 10);
        softBody_->solveConstraints();
    }
    std::cerr << std::endl;

    // Relax the springs
    world_->setInternalTickCallback(
        axisLockCallback, static_cast<void*>(this), true);
    world_->setGravity(btVector3(0.0f, static_cast<float>(expansionA_), 0.0f));
    softBody_->getWorldInfo()->m_gravity = world_->getGravity();
    softBody_->m_cfg.kDP = 0.1f;
    int counter = 0;
    double relativeError = std::fabs(
        (startingSurfaceArea_ - surface_area_()) / startingSurfaceArea_);
    while (relativeError > 0.0) {
        std::cerr << "volcart::texturing::clothUV: Relaxing " << counter + 1
                  << "\r" << std::flush;
        world_->stepSimulation(1 / 60.f, 10);
        softBody_->solveConstraints();

        ++counter;
        if (counter % 10 == 0) {
            relativeError = std::fabs(
                (startingSurfaceArea_ - surface_area_()) /
                startingSurfaceArea_);
        }
        if (counter >= expandIterations_ * 6) {
            std::cerr << std::endl
                      << "volcart::texturing::clothUV: Warning: Max relaxation "
                         "iterations reached";
            break;
        }
    }
    std::cerr << std::endl
              << "volcart::texturing::clothUV: Mesh Area Relative Error: "
              << relativeError * 100 << "%" << std::endl;
}

// Set acceleration for different stages
void ClothModelingUVMapping::setAcceleration(Stage s, double a)
{
    switch (s) {
        case Stage::Unfurl:
            unfurlA_ = a;
            break;
        case Stage::Collision:
            collisionA_ = a;
            break;
        case Stage::Expansion:
            expansionA_ = a;
            break;
    }
}

///// Helper Functions /////

// Calculate the surface area of the mesh using Heron's formula
// Use the version that is stable for small angles from
// "Miscalculating Area and Angles of a Needle-like Triangle" by Kahan
// https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
double ClothModelingUVMapping::surface_area_()
{
    double surfaceArea = 0;
    for (int i = 0; i < softBody_->m_faces.size(); ++i) {

        // Get the side lengths
        double a = 0, b = 0, c = 0, p = 0;
        a = softBody_->m_faces[i].m_n[0]->m_x.distance(
            softBody_->m_faces[i].m_n[1]->m_x);
        b = softBody_->m_faces[i].m_n[0]->m_x.distance(
            softBody_->m_faces[i].m_n[2]->m_x);
        c = softBody_->m_faces[i].m_n[1]->m_x.distance(
            softBody_->m_faces[i].m_n[2]->m_x);

        // Sort the side lengths so that a >= b >= c
        double na, nb, nc;
        nc = std::min(a, std::min(b, c));
        na = std::max(a, std::max(b, c));
        nb = a + b + c - na - nc;

        // Calculate the area
        p = (na + (nb + nc)) * (nc - (na - nb)) * (nc + (na - nb)) *
            (na + (nb - nc));
        double sa = 0.25 * sqrt(p);

        // Can get NaN's when using standard C++ math. Explore something
        // like
        // GMP
        if (std::isnan(sa)) {
            std::cerr << std::endl
                      << "volcart::texturing::clothUV: Warning: NaN surface "
                         "area for face["
                      << i << "]. Evaluating as 0." << std::endl;
            sa = 0.0;
        }
        surfaceArea += sa;
    }

    return surfaceArea;
}

///// Callback Functions /////
// Limits motion to be along the X axis only. Used in unfurl step.
void ClothModelingUVMapping::cbConstrainMotion(btScalar /*timeStep*/)
{
    for (auto n = 0; n < softBody_->m_nodes.size(); ++n) {
        btVector3 velocity = softBody_->m_nodes[n].m_v;
        velocity.setY(0);
        velocity.setZ(0);
        softBody_->m_nodes[n].m_v = velocity;
    }
}

// Apply opposite velocity to points that have "passed through" the
// collision plane
void ClothModelingUVMapping::cbAxisLock(btScalar /*timeStep*/)
{
    for (auto n = 0; n < softBody_->m_nodes.size(); ++n) {
        btVector3 pos = softBody_->m_nodes[n].m_x;
        if (pos.getY() < 0.0) {
            btVector3 velocity = softBody_->m_nodes[n].m_v;
            velocity.setY(velocity.getY() * -1.5f);  // push it back some
            softBody_->m_nodes[n].m_v = velocity;
        }
    }
}

// Move points in the currentPins_ vector towards their respective targets
void ClothModelingUVMapping::cbMoveTowardTarget(btScalar timeStep)
{
    for (auto pin = currentPins_.begin(); pin < currentPins_.end(); ++pin) {
        btVector3 delta = (pin->target - pin->node->m_x).normalized();
        pin->node->m_v += delta / timeStep;
    }
}
// This call back is used to disable other callbacks
void ClothModelingUVMapping::cbEmptyPreTick(btScalar /*timeStep*/)
{
    // Don't do anything
}
