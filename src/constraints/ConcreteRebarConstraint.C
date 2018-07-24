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

registerMooseObject("BlackBearApp", ConcreteRebarConstraint);

template <>
InputParameters
validParams<EqualValueEmbeddedConstraint>()
{
  InputParameters params = validParams<NodeElemConstraint>();
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
    _model(getModel("model"))
{
  // modern parameter scheme for displacements
  for (unsigned int i = 0; i < coupledComponents("displacements"); ++i)
  {
    _var_nums[i] = coupled("displacements", i);
    _vars[i] = getVar("displacements", i);
  }
}

void
ConcreteRebarConstraint::computeConstraintForce()
{
  // compute constraint force once per constraint
  if (_component != 0)
    return;

  const Node * node = _current_node;
  unsigned int sys_num = _sys.number();

  // Build up residual vector and penalty force
  RealVectorValue res_vec;
  for (unsigned int i = 0; i < _mesh_dimension; ++i)
  {
    dof_id_type dof_number = node->dof_number(sys_num, _var_nums[i], 0);
    res_vec(i) = _residual_copy(dof_number) / _vars[i]->scalingFactor();
    _pen_force(i) = _penalty * ((_vars[i]->dofValues())[_qp] - (_vars[i]->slnNeighbor())[_qp]);
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
          curr_jac = (*_jacobian)(_current_node->dof_number(sys_num, _var.number(), 0),
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
          slave_jac = (*_jacobian)(_current_node->dof_number(sys_num, _var.number(), 0),
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
      curr_jac = (*_jacobian)(_current_node->dof_number(sys_num, _var.number(), 0),
                              _connected_dof_indices[_j]);
      return -curr_jac;

    case Moose::SlaveMaster:
      return 0.0;

    case Moose::MasterSlave:
      switch (_formulation)
      {
        case KINEMATIC:
          slave_jac = (*_jacobian)(_current_node->dof_number(sys_num, _var.number(), 0),
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
