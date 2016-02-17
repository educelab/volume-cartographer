//
// Created by Abigail Coleman 2/3/16
//

#include "cloth.h"

namespace volcart {
    namespace meshing {

        cloth::cloth ( VC_MeshType::Pointer inputMesh,
                   VC_MeshType::Pointer decimated,
                   int width,
                   int height,
                   int required_iterations) :
                   _input(inputMesh), _decimated(decimated), _width(width), _height(height), _iterations(required_iterations)
        {
      _process();
    };

    int cloth::_process() {
        // Create Dynamic world for bullet cloth simulation
        btBroadphaseInterface* broadphase = new btDbvtBroadphase();

        btDefaultCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

        btSoftBodySolver* softBodySolver = new btDefaultSoftBodySolver();

        btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld( dispatcher,
                                                                            broadphase,
                                                                            solver,
                                                                            collisionConfiguration,
                                                                            softBodySolver);

        dynamicsWorld->setGravity(btVector3(0, 0, 0));

        // convert itk mesh to bullet mesh (vertices and triangle arrays)
        volcart::meshing::itk2bullet::itk2bullet(_decimated, dynamicsWorld->getWorldInfo(), &_psb);

        _psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity
        dynamicsWorld->addSoftBody(_psb);
        _dumpState( _decimated, _psb, "_0" );

        // Constraints for the mesh as a soft body
        // These needed to be tested to find optimal values.
        // Sets the mass of the whole soft body, true considers the faces along with the vertices
        // Note: Mass is in kilograms. If mass isn't high enough, nothing changes.
        printf("volcart::cloth::message: Setting mass\n");
        _psb->setTotalMass( (int)(_psb->m_nodes.size() * 0.001), true );

        _psb->m_cfg.kDP = 0.01; // Damping coefficient of the soft body [0,1]
        _psb->m_materials[0]->m_kLST = 1.0; // Linear stiffness coefficient [0,1]
        _psb->m_materials[0]->m_kAST = 1.0; // Area/Angular stiffness coefficient [0,1]
        _psb->m_materials[0]->m_kVST = 1.0; // Volume stiffness coefficient [0,1]

        // Find the position of the four corner nodes
        // Currently assumes that the first point has the same z-value as the rest of the starting chain
        // This needs work. A lot of work.
        int min_z = (int) std::floor(_input->GetPoint(0)[2]);
        unsigned long chain_size = 1;
        double chain_length = 0;
        // Calculate chain size and chain length
        for(unsigned long i = 1; i < _input->GetNumberOfPoints(); ++i) {
          if( _input->GetPoint(i)[2] <= min_z ) {
            chain_length += _input->GetPoint(i).EuclideanDistanceTo(_input->GetPoint(i-1));
            ++chain_size;
          }
          else
            break;
        }
        VC_PointType tl = _input->GetPoint(0);
        VC_PointType tr = _input->GetPoint(chain_size - 1);
        VC_PointType bl = _input->GetPoint(_input->GetNumberOfPoints() - chain_size);
        VC_PointType br = _input->GetPoint(_input->GetNumberOfPoints() - 1);

        unsigned long tl_id, tr_id, bl_id, br_id;
        tl_id = tr_id = bl_id = br_id = 0;
        for ( auto pt = _decimated->GetPoints()->Begin(); pt != _decimated->GetPoints()->End(); ++pt ) {
          if ( tl.EuclideanDistanceTo(pt->Value()) < tl.EuclideanDistanceTo(_decimated->GetPoint(tl_id)) ) tl_id = pt->Index();
          if ( tr.EuclideanDistanceTo(pt->Value()) < tr.EuclideanDistanceTo(_decimated->GetPoint(tr_id)) ) tr_id = pt->Index();
          if ( bl.EuclideanDistanceTo(pt->Value()) < bl.EuclideanDistanceTo(_decimated->GetPoint(bl_id)) ) bl_id = pt->Index();
          if ( br.EuclideanDistanceTo(pt->Value()) < br.EuclideanDistanceTo(_decimated->GetPoint(br_id)) ) br_id = pt->Index();
        }

        // For debug. These values should match.
        if ( chain_size != _width ) return EXIT_FAILURE;

        // Append rigid bodies to respective nodes of mesh
        // Assumes the chain length is constant throughout the mesh
        btSoftBody::Node* top_left     = &_psb->m_nodes[tl_id];
        btSoftBody::Node* top_right    = &_psb->m_nodes[tr_id];
        btSoftBody::Node* bottom_left  = &_psb->m_nodes[bl_id];
        btSoftBody::Node* bottom_right = &_psb->m_nodes[br_id];

        _pinnedPoints.push_back(top_left);
        _pinnedPoints.push_back(top_right);
        _pinnedPoints.push_back(bottom_left);
        _pinnedPoints.push_back(bottom_right);

        double pinMass = 10;
        _psb->setMass(tl_id, pinMass);
        _psb->setMass(tr_id, pinMass);
        _psb->setMass(bl_id, pinMass);
        _psb->setMass(br_id, pinMass);

        // Calculate the surface area of the mesh
        double surface_area = _btSurfaceArea(_psb);
        int dir = ( top_left->m_x.getX() < top_right->m_x.getX() ) ? 1 : -1;
        double width = chain_length * dir;
        double height = bl[2] - tl[2];
        int required_iterations = _iterations; // Minimum iterations to reach target

        // Create target positions with step size for our four corners
        // NOTE: Must be created in the same order that the rigid bodies were put into pinnedPoints
        NodeTarget n_target;
        btScalar t_x, t_y, t_z;
        btSoftBody::Node* node_ptr;

        // top left corner
        node_ptr = &_psb->m_nodes[tl_id];
        n_target.t_pos = _psb->m_nodes[tl_id].m_x;
        n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
        _targetPoints.push_back(n_target);

        // top right corner
        node_ptr = &_psb->m_nodes[tr_id];
        t_x = _psb->m_nodes[tl_id].m_x.x() + width;
        t_y = _psb->m_nodes[tl_id].m_x.y();
        t_z = _psb->m_nodes[tl_id].m_x.z();
        n_target.t_pos = btVector3(t_x, t_y, t_z);
        n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
        _targetPoints.push_back(n_target);

        // bottom left corner
        node_ptr = &_psb->m_nodes[bl_id];
        t_x = _psb->m_nodes[tl_id].m_x.x();
        t_y = _psb->m_nodes[tl_id].m_x.y();
        t_z = _psb->m_nodes[tl_id].m_x.z() + height;
        n_target.t_pos = btVector3(t_x, t_y, t_z);
        n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
        _targetPoints.push_back(n_target);

        // bottom right corner
        node_ptr = &_psb->m_nodes[br_id];
        t_x = _psb->m_nodes[tl_id].m_x.x() + width;
        t_y = _psb->m_nodes[tl_id].m_x.y();
        t_z = _psb->m_nodes[tl_id].m_x.z() + height;
        n_target.t_pos = btVector3(t_x, t_y, t_z);
        n_target.t_stepsize = (node_ptr)->m_x.distance(n_target.t_pos) / required_iterations;
        _targetPoints.push_back(n_target);

        // Middle of target rectangle
        t_x = _psb->m_nodes[tl_id].m_x.x() + (width / 2);
        t_y = _psb->m_nodes[tl_id].m_x.y();
        t_z = _psb->m_nodes[tl_id].m_x.z() + (height / 2);
        _middle = btVector3(t_x, t_y, t_z);

        // Planarize the corners
        printf("volcart::cloth::message: Planarizing corners\n");
        dynamicsWorld->setInternalTickCallback(_planarizeCornersPreTickCallback, dynamicsWorld, true);
        int counter = 0;
        while ( counter < required_iterations ) {
          std::cerr << "volcart::cloth::message: Step " << counter+1 << "/" << required_iterations << "\r" << std::flush;
          dynamicsWorld->stepSimulation(1/60.f);
          _psb->solveConstraints();
          ++counter;
        }
        std::cerr << std::endl;
        printf("Planarize steps: %d\n", counter);
        _dumpState( _decimated, _psb, "_1");

        // Expand the corners
        printf("volcart::cloth::message: Expanding corners\n");
        counter = 0;
        required_iterations = required_iterations * 2;
        while ( (_btAverageNormal(_psb).absolute().getY() < 0.925 || counter < required_iterations) && counter < required_iterations*2 ) {
          std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
          if ( counter % 2000 == 0 ) _expandCorners( 10 + (counter / 2000) );
          dynamicsWorld->stepSimulation(1/60.f);
          _psb->solveConstraints();
          ++counter;
        }
        std::cerr << std::endl;
        printf("Expansion steps: %d\n", counter);
        _dumpState( _decimated, _psb, "_2" );

        // Add a collision plane to push the mesh onto
        btScalar min_y = _psb->m_nodes[0].m_x.y();
        btScalar max_y = _psb->m_nodes[0].m_x.y();
        for (size_t n_id = 1; n_id < _psb->m_nodes.size(); ++n_id) {
            double _y = _psb->m_nodes[n_id].m_x.y();
            double _z = _psb->m_nodes[n_id].m_x.z();
            if ( _y < min_y && _z >= 0) min_y = _psb->m_nodes[n_id].m_x.y();
            if ( _y > max_y && _z >= 0) max_y = _psb->m_nodes[n_id].m_x.y();
        }

        btScalar plane_y = min_y - 5;
        btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, plane_y, 0), 1);
        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
        btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
        btRigidBody* plane = new btRigidBody(groundRigidBodyCI);
        dynamicsWorld->addRigidBody(plane);

        // Set the gravity so the mesh will be pushed onto the plane
        dynamicsWorld->setGravity(btVector3(0, -15, 0));
        _psb->getWorldInfo()->m_gravity = dynamicsWorld->getGravity(); // Have to explicitly make softbody gravity match world gravity

        // set the friction of the plane and the mesh s.t. the mesh can easily flatten upon collision
        plane->setFriction(0.01); // (0-1] Default: 0.5
        _psb->m_cfg.kDF = 0.01; // Dynamic friction coefficient (0-1] Default: 0.2
        _psb->m_cfg.kDP = 0.1; // Damping coefficient of the soft body [0,1]

        // Let it settle
        printf("volcart::cloth::message: Relaxing corners\n");
        dynamicsWorld->setInternalTickCallback(_emptyPreTickCallback, dynamicsWorld, true);
        required_iterations = required_iterations * 2;
        counter = 0;
        double test_area = _btSurfaceArea(_psb);
        while ( (isnan(test_area) || test_area/surface_area > 1.001 || counter < required_iterations) && counter < required_iterations*4 ) {
          std::cerr << "volcart::cloth::message: Step " << counter+1 << "\r" << std::flush;
          dynamicsWorld->stepSimulation(1/60.f);
          _psb->solveConstraints();

          ++counter;
          if ( counter % 500 == 0 ) test_area = _btSurfaceArea(_psb); // recalc area every 500 iterations
        }
        std::cerr << std::endl;
        printf("Relaxation steps: %d\n", counter);
        _dumpState( _decimated, _psb, "_3" );

        // bullet clean up
        dynamicsWorld->removeRigidBody(plane);
        delete plane->getMotionState();
        delete plane;
        delete groundShape;
        dynamicsWorld->removeSoftBody(_psb);
        // delete _psb;
        delete dynamicsWorld;
        delete softBodySolver;
        delete solver;
        delete dispatcher;
        delete collisionConfiguration;
        delete broadphase;

        return EXIT_SUCCESS;
    };

    btVector3 cloth::_btAverageNormal(btSoftBody* body) {
      btVector3  avg_normal(0,0,0);
      for ( size_t n_id = 0; n_id < body->m_faces.size(); ++n_id ) {
        avg_normal += body->m_faces[n_id].m_normal;
      }
      avg_normal /= body->m_faces.size();
      return avg_normal;
    };

    btScalar cloth::_btAverageVelocity(btSoftBody* body) {
      btScalar velocity = 0;
      for ( size_t n_id = 0; n_id < body->m_nodes.size(); ++n_id ) {
        velocity += body->m_nodes[n_id].m_v.length();
      }
      velocity /= body->m_nodes.size();
      return velocity;
    };

    // Calculate the surface area of the mesh using Heron's formula
    // Let a,b,c be the lengths of the sides of a triangle and p the semiperimeter
    // p = (a +  b + c) / 2
    // area of triangle = sqrt( p * (p - a) * (p - b) * (p - c) )
    double cloth::_btSurfaceArea( btSoftBody* body ) {
      double surface_area = 0;
      for(int i = 0; i < body->m_faces.size(); ++i) {
        double a = 0, b = 0, c = 0, p = 0;
        a = body->m_faces[i].m_n[0]->m_x.distance(body->m_faces[i].m_n[1]->m_x);
        b = body->m_faces[i].m_n[0]->m_x.distance(body->m_faces[i].m_n[2]->m_x);
        c = body->m_faces[i].m_n[1]->m_x.distance(body->m_faces[i].m_n[2]->m_x);

        p = (a + b + c) / 2;

        surface_area += sqrt( p * (p - a) * (p - b) * (p - c) );
      }
      return surface_area;
    };

    void cloth::_planarizeCornersPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
      // Iterate over rigid bodies and move them towards their targets
      for( size_t p_id = 0; p_id < _pinnedPoints.size(); ++p_id ) {
        if ( _pinnedPoints[p_id]->m_x == _targetPoints[p_id].t_pos ) continue;
        btVector3 delta = (_targetPoints[p_id].t_pos - _pinnedPoints[p_id]->m_x).normalized() * _targetPoints[p_id].t_stepsize;
        _pinnedPoints[p_id]->m_v += delta/timeStep;
      }
    };

    void cloth::_emptyPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
      // This call back is used to disable other callbacks
      // Particularly used for relaxing the four corners
    };

    void cloth::_expandCorners(float magnitude) {
      btScalar stepSize = 1;
      btScalar _magnitude = magnitude;

      for( size_t p_id = 0; p_id < _pinnedPoints.size(); ++p_id ) {
        _targetPoints[p_id].t_pos += (_targetPoints[p_id].t_pos - _middle).normalized() * _magnitude;
        _targetPoints[p_id].t_stepsize = stepSize;
      }
    };

    void cloth::_dumpState( VC_MeshType::Pointer toUpdate, btSoftBody* body, std::string suffix ) {
      VC_MeshType::Pointer output = VC_MeshType::New();
      volcart::meshing::deepCopy(toUpdate, output);
      volcart::meshing::bullet2itk::bullet2itk(body, output);

      std::string path = "inter_" + VC_DATE_TIME() + suffix + ".obj";
      volcart::io::objWriter writer(path, output);
      writer.write();
    };

    volcart::UVMap cloth::_returnUVMap() {
      // UV map setup
      double min_u = _psb->m_nodes[0].m_x.x();
      double min_v = _psb->m_nodes[0].m_x.z();
      double max_u = _psb->m_nodes[0].m_x.x();
      double max_v = _psb->m_nodes[0].m_x.z();

      for (size_t n_id = 0; n_id < _psb->m_nodes.size(); ++n_id) {
        double _x = _psb->m_nodes[n_id].m_x.x();
        double _z = _psb->m_nodes[n_id].m_x.z();
        if ( _x < min_u && _z >= 0) min_u = _psb->m_nodes[n_id].m_x.x();
        if ( _z < min_v && _z >= 0) min_v = _psb->m_nodes[n_id].m_x.z();
        if ( _x > max_u && _z >= 0) max_u = _psb->m_nodes[n_id].m_x.x();
        if ( _z > max_v && _z >= 0) max_v = _psb->m_nodes[n_id].m_x.z();
      }

      // Round so that we have integer bounds
      min_u = std::floor(min_u);
      min_v = std::floor(min_v);
      max_u = std::ceil(max_u);
      max_v = std::ceil(max_v);

      double aspect_width = max_u - min_u;
      double aspect_height = max_v - min_v;
      double aspect = aspect_width / aspect_height;
      volcart::UVMap uvMap;
      uvMap.ratio(aspect_width, aspect_height);

      // Calculate uv coordinates
      double u, v;
      for (size_t f_id = 0; f_id < _psb->m_faces.size(); ++f_id) {

        for(size_t n_id = 0; n_id < 3; ++n_id) {

          u = (_psb->m_faces[f_id].m_n[n_id]->m_x.x() - min_u) / (max_u - min_u);
          v = (_psb->m_faces[f_id].m_n[n_id]->m_x.z() - min_v) / (max_v - min_v);
          cv::Vec2d uv( u, v );

          // btSoftBody faces hold pointers to specific nodes, but we need the point id
          // Lookup the point ID of this node in the original ITK mesh
          VC_CellType::CellAutoPointer c;
          _decimated->GetCell(f_id, c);
          double p_id = c->GetPointIdsContainer()[n_id];

          // Add the uv coordinates into our map at the point index specified
          uvMap.set(p_id, uv);

        }
      }

      _uvMap = uvMap;
      return ( _uvMap );
    };

    } // namespace meshing
} // namespace volcart