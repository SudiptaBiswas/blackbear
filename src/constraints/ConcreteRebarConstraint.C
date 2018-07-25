//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html

// MOOSE includes
#include "ConcreteRebarConstraint.h"
#include "libmesh/string_to_enum.h"

registerMooseObject("BlackBearApp", ConcreteRebarConstraint);

template <>
InputParameters
validParams<ConcreteRebarConstraint>()
{
  InputParameters params = validParams<EqualValueEmbeddedConstraint>();
  params.addClassDescription("This is a constraint enforcing concrete-rebar contact");
  params.addRequiredParam<unsigned int>("component",
                                        "An integer corresponding to the direction "
                                        "the variable this kernel acts in. (0 for x, "
                                        "1 for y, 2 for z)");
  params.addCoupledVar(
      "displacements",
      "The displacements appropriate for the simulation geometry and coordinate system");
  params.addRequiredParam<std::string>("model", "model used for enforcing the constraint");
  return params;
}

ConcreteRebarConstraint::ConcreteRebarConstraint(const InputParameters & parameters)
  : EqualValueEmbeddedConstraint(parameters),
    _component(getParam<unsigned int>("component")),
    _mesh_dimension(_mesh.dimension()),
    _var_nums(_mesh_dimension, libMesh::invalid_uint),
    _vars(_mesh_dimension, nullptr),
    _model(getModel(getParam<std::string>("model")))
{
  if (_mesh_dimension != coupledComponents("displacements"))
    mooseError("In ConcreteRebarConstraint, number of displacements must equal the mesh dimension");
  // modern parameter scheme for displacements
  for (unsigned int i = 0; i < _mesh_dimension; ++i)
  {
    _var_nums[i] = coupled("displacements", i);
    _vars[i] = getVar("displacements", i);
  }
}

void
ConcreteRebarConstraint::computeConstraintForce()
{
  // get connected elements of the current node
  auto node_to_elem_pair = _node_to_elem_map.find(_current_node->id());
  mooseAssert(node_to_elem_pair != _node_to_elem_map.end(), "Missing entry in node to elem map");
  const std::vector<dof_id_type> & elems = node_to_elem_pair->second;
  std::cout << "found connected elems" << std::endl;
  std::cout << "number of connected elems = " << elems.size() << std::endl;
  dof_id_type elem = elems[0];
  std::cout << "elem dof = " << elem << std::endl;
  Elem * elem_ptr = _mesh.elemPtr(elem);
  std::cout << "prepared elem pointer" << std::endl;
  const std::vector<Point> pt = {*_current_node};

  // calculate phi and dphi for this element
  FEType fe_type(Utility::string_to_enum<Order>("first"),
                 Utility::string_to_enum<FEFamily>("lagrange"));
  std::cout << "prepared fe type" << std::endl;
  std::unique_ptr<FEBase> fe(FEBase::build(_mesh_dimension, fe_type));
  std::cout << "built fe base" << std::endl;
  // fe->attach_quadrature_rule(_assembly.qRule());
  const std::vector<std::vector<Point>> * tangents = &fe->get_tangents();
  std::cout << "about to reinit fe" << std::endl;
  fe->reinit(elem_ptr, &pt);

  // debug
  std::cout << "===========================================\n";
  std::cout << "node id: " << _current_node->id() << std::endl;
  std::cout << "at coord: " << (Point)*_current_node << std::endl;
  std::cout << "tangents.size() " << tangents->size() << std::endl;
  std::cout << "tangents[0].size() " << (*tangents)[0].size() << std::endl;

  // compute constraint force once per constraint
  // if (_component != 0)
  //   return;

  const Node * node = _current_node;
  unsigned int sys_num = _sys.number();

  // Build up residual vector and penalty force
  RealVectorValue res_vec;
  for (unsigned int i = 0; i < _mesh_dimension; ++i)
  {
    // std::cout << "i = " << i << std::endl;
    dof_id_type dof_number = node->dof_number(sys_num, _var_nums[i], 0);
    // std::cout << "dof = " << dof_number << std::endl;
    res_vec(i) = _residual_copy(dof_number);
    // std::cout << "res[i] = " << res_vec(i) << std::endl;
    // std::cout << "_vars.size() = " << _vars.size() << std::endl;
    // std::cout << "_vars[i]->dofValues().size() = " << _vars[i]->dofValues().size() << std::endl;
    // std::cout << "_qp = " << _qp << std::endl;
    // std::cout << "u_slave[i] = " << (_vars[i]->dofValues())[_qp] << std::endl;
    // std::cout << "u_master[i] = " << (_vars[i]->slnNeighbor())[_qp] << std::endl;
    _pen_force(i) = _penalty * ((_vars[i]->dofValues())[0] - (_vars[i]->slnNeighbor())[0]);
    // std::cout << "pen_force[i] = " << _pen_force(i) << std::endl;
  }

  switch (_model)
  {
    case GLUED:
      switch (_formulation)
      {
        case KINEMATIC:
          _constraint_force = -res_vec;
          break;
        case PENALTY:
          _constraint_force = _pen_force;
          break;
        default:
          mooseError("Invalid formulation");
          break;
      }
      break;
    case BONDSLIP:
    default:
      mooseError("Invalid model");
      break;
  }
}

Real
ConcreteRebarConstraint::computeQpResidual(Moose::ConstraintType type)
{
  Real resid = _constraint_force(_component);

  switch (type)
  {
    case Moose::Slave:
    {
      if (_formulation == KINEMATIC)
        resid += _pen_force(_component);
      else if (_model == BONDSLIP)
        resid += 0.0;
      return _test_slave[_i][_qp] * resid;
    }

    case Moose::Master:
      return _test_master[_i][_qp] * -resid;
  }

  return 0.0;
}

Real
ConcreteRebarConstraint::computeQpJacobian(Moose::ConstraintJacobianType type)
{
  unsigned int sys_num = _sys.number();
  const Real penalty = _penalty;
  Real curr_jac, slave_jac;

  switch (type)
  {
    case Moose::SlaveSlave:
      switch (_formulation)
      {
        case KINEMATIC:
          curr_jac = (*_jacobian)(_current_node->dof_number(sys_num, _var_nums[_component], 0),
                                  _connected_dof_indices[_j]);
          return -curr_jac + _phi_slave[_j][_qp] * penalty * _test_slave[_i][_qp];
        case PENALTY:
          return _phi_slave[_j][_qp] * penalty * _test_slave[_i][_qp];
        default:
          mooseError("Invalid formulation");
      }

    case Moose::SlaveMaster:
      switch (_formulation)
      {
        case KINEMATIC:
          return -_phi_master[_j][_qp] * penalty * _test_slave[_i][_qp];
        case PENALTY:
          return -_phi_master[_j][_qp] * penalty * _test_slave[_i][_qp];
        default:
          mooseError("Invalid formulation");
      }

    case Moose::MasterSlave:
      switch (_formulation)
      {
        case KINEMATIC:
          slave_jac = (*_jacobian)(_current_node->dof_number(sys_num, _var_nums[_component], 0),
                                   _connected_dof_indices[_j]);
          return slave_jac * _test_master[_i][_qp];
        case PENALTY:
          return -_phi_slave[_j][_qp] * penalty * _test_master[_i][_qp];
        default:
          mooseError("Invalid formulation");
      }

    case Moose::MasterMaster:
      switch (_formulation)
      {
        case KINEMATIC:
          return 0.0;
        case PENALTY:
          return _test_master[_i][_qp] * penalty * _phi_master[_j][_qp];
        default:
          mooseError("Invalid formulation");
      }
  }
  return 0.0;
}

Real
ConcreteRebarConstraint::computeQpOffDiagJacobian(Moose::ConstraintJacobianType type,
                                                  unsigned int /*jvar*/)
{
  Real curr_jac, slave_jac;
  unsigned int sys_num = _sys.number();

  switch (type)
  {
    case Moose::SlaveSlave:
      curr_jac = (*_jacobian)(_current_node->dof_number(sys_num, _var_nums[_component], 0),
                              _connected_dof_indices[_j]);
      return -curr_jac;

    case Moose::SlaveMaster:
      return 0.0;

    case Moose::MasterSlave:
      switch (_formulation)
      {
        case KINEMATIC:
          slave_jac = (*_jacobian)(_current_node->dof_number(sys_num, _var_nums[_component], 0),
                                   _connected_dof_indices[_j]);
          return slave_jac * _test_master[_i][_qp];
        case PENALTY:
          return 0.0;
        default:
          mooseError("Invalid formulation");
      }

    case Moose::MasterMaster:
      return 0.0;
  }

  return 0.0;
}

Model
ConcreteRebarConstraint::getModel(std::string name)
{
  Model model(MODEL_INVALID);
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);

  if ("glued" == name)
    model = GLUED;

  else if ("bondslip" == name)
    model = BONDSLIP;

  if (model == MODEL_INVALID)
    ::mooseError("Invalid model found: ", name);

  return model;
}
