//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html

// MOOSE includes
#include "RebarBondSlipConstraint.h"
#include "Assembly.h"
#include "SystemBase.h"
#include "FEProblem.h"
#include "MathUtils.h"
#include "Executioner.h"

#include "libmesh/string_to_enum.h"

registerMooseObject("BlackBearApp", RebarBondSlipConstraint);

defineLegacyParams(RebarBondSlipConstraint);

InputParameters
RebarBondSlipConstraint::validParams()
{
  InputParameters params = EqualValueEmbeddedConstraint::validParams();
  params.addClassDescription(
      "This is a constraint enforcing the bodslip behavior between concrete and rebar");
  params.addRequiredParam<unsigned int>("component",
                                        "An integer corresponding to the direction "
                                        "the variable this kernel acts in. (0 for x, "
                                        "1 for y, 2 for z)");
  params.addCoupledVar(
      "displacements",
      "The displacements appropriate for the simulation geometry and coordinate system");
  params.addParam<bool>("debug", false, "whether to print out debug messages");
  params.addParam<Real>("max_bondstress", 0.0, "Maximum bond stress");
  params.addParam<Real>("frictional_bondstress", 0.0, "Bond stress due to friction");

  params.addParam<std::vector<Real>>(
      "transitional_slip_values",
      "Singificant slip values at which the bondstress curve changes pattern/slope or "
      "trnsitions to a different function");
  params.addParam<Real>(
      "ultimate_slip", 0.05, "Ultimate value of slip at which the concrete and rebar debonds");
  params.addParam<Real>("rebar_radius", 1.0, "Radius of the rebar");
  return params;
}

RebarBondSlipConstraint::RebarBondSlipConstraint(const InputParameters & parameters)
  : EqualValueEmbeddedConstraint(parameters),
    _component(getParam<unsigned int>("component")),
    _mesh_dimension(_mesh.dimension()),
    _var_nums(_mesh_dimension, libMesh::invalid_uint),
    _vars(_mesh_dimension, nullptr),
    _debug(getParam<bool>("debug")),
    _max_bondstress(getParam<Real>("max_bondstress")),
    _frictional_bondstress(getParam<Real>("frictional_bondstress")),
    _ultimate_slip(getParam<Real>("ultimate_slip")),
    _bar_radius(getParam<Real>("rebar_radius")),
    _transitional_slip(getParam<std::vector<Real>>("transitional_slip_values"))
{
  if (_mesh_dimension != coupledComponents("displacements"))
    mooseError("In RebarBondSlipConstraint, number of displacements must equal the mesh dimension");

  for (unsigned int i = 0; i < _mesh_dimension; ++i)
  {
    _var_nums[i] = coupled("displacements", i);
    _vars[i] = getVar("displacements", i);
  }
}

void
RebarBondSlipConstraint::initialSetup()
{
  for (auto it = _slave_to_master_map.begin(); it != _slave_to_master_map.end(); ++it)
    if (_bondslip.find(it->first) == _bondslip.end())
      _bondslip.insert(std::pair<dof_id_type, bondSlipData>(it->first,
                                                            bondSlipData())); // initialize
}

void
RebarBondSlipConstraint::timestepSetup()
{
  if (_t_step > 1)
    if (_app.getExecutioner()->lastSolveConverged())
      for (auto iter = _slave_to_master_map.begin(); iter != _slave_to_master_map.end(); ++iter)
      {
        dof_id_type node_id = iter->first;
        auto it = _bondslip.find(node_id);
        mooseAssert(it != _bondslip.end(), "Node not found in bond-slip map");

        _bondslip[node_id].slip_min_old = _bondslip[node_id].slip_min;
        _bondslip[node_id].slip_max_old = _bondslip[node_id].slip_max;
        _bondslip[node_id].bondstress_min_old = _bondslip[node_id].bondstress_min;
        _bondslip[node_id].bondstress_max_old = _bondslip[node_id].bondstress_max;
      }
}

bool
RebarBondSlipConstraint::shouldApply()
{
  if (_debug)
  {
    if (_current_node->id() == 6 || _current_node->id() == 124 || _current_node->id() == 140)
    {
      std::cout << "===========================================\n";
      std::cout << "node id: " << _current_node->id() << std::endl;
      // std::cout << "at coord: " << (Point)*_current_node << std::endl;
    }
  }
  auto it = _slave_to_master_map.find(_current_node->id());

  if (it != _slave_to_master_map.end())
  {
    const Elem * master_elem = _mesh.elemPtr(it->second);
    std::vector<Point> points = {*_current_node};

    // reinit variables on the master element at the slave point
    _fe_problem.setNeighborSubdomainID(master_elem, 0);
    _fe_problem.reinitNeighborPhys(master_elem, points, 0);

    reinitConstraint();

    return true;
  }
  return false;
}

void
RebarBondSlipConstraint::computeTangent()
{
  _slave_tangent = 0.0;

  // get normals
  // get connected elements of the current node
  const std::map<dof_id_type, std::vector<dof_id_type>> & node_to_elem_map = _mesh.nodeToElemMap();
  auto node_to_elem_pair = node_to_elem_map.find(_current_node->id());
  mooseAssert(node_to_elem_pair != node_to_elem_map.end(), "Missing entry in node to elem map");
  const std::vector<dof_id_type> & elems = node_to_elem_pair->second;

  for (auto & elem : elems)
  {
    Elem * elem_ptr = _mesh.elemPtr(elem);
    _assembly.reinit(elem_ptr, 0);
    _current_elem_volume += _assembly.elemVolume();
    // calculate phi and dphi for this element
    FEType fe_type(Utility::string_to_enum<Order>("first"),
                   Utility::string_to_enum<FEFamily>("lagrange"));
    std::unique_ptr<FEBase> fe(FEBase::build(1, fe_type));
    fe->attach_quadrature_rule(const_cast<QBase *>(_assembly.qRule()));
    const std::vector<RealGradient> * tangents = &fe->get_dxyzdxi();
    unsigned side = 0;
    fe->reinit(elem_ptr, side);
    for (unsigned i = 0; i < tangents->size(); i++)
      _slave_tangent += (*tangents)[i];
  }

  _slave_tangent /= _slave_tangent.norm();
  _current_elem_volume /= elems.size();

  // if (_debug)
  //   if (_current_node->id() == 6)
  //     std::cout << "tangent: " << _slave_tangent << std::endl;
}

void
RebarBondSlipConstraint::reinitConstraint()
{
  computeTangent();

  // Build up residual vector
  RealVectorValue relative_disp;
  for (unsigned int i = 0; i < _mesh_dimension; ++i)
    relative_disp(i) = ((_vars[i]->dofValues())[0] - (_vars[i]->slnNeighbor())[0]);

  Real slip = relative_disp * _slave_tangent;
  RealVectorValue slip_axial = slip * _slave_tangent;
  RealVectorValue slip_normal = relative_disp - slip_axial;
  const Real slip_ratio = slip / (MathUtils::sign(slip) * _transitional_slip[0]);

  const Node * node = _current_node;
  auto it = _bondslip.find(node->id());
  mooseAssert(it != _bondslip.end(), "Node not found in bond-slip map");
  bondSlipData bond_slip = it->second;

  // Real bond_stress;
  if (_debug)
    if (node->id() == 6 || node->id() == 124 || node->id() == 140)
    {
      // std::cout << "Slave_disp = " << _vars[0]->dofValues())[0] << ".\n";
      // std::cout << "Relative_disp = " << relative_disp << ".\n";
      std::cout << "Slip = " << slip << ".\n";
    }

  bond_slip.slip_min = std::min(bond_slip.slip_min_old, slip);
  bond_slip.slip_max = std::max(bond_slip.slip_max_old, slip);

  // if (_debug)
  //   if (_current_node->id() == 6)
  //   {
  //     std::cout << "slip_ratio = " << slip_ratio << ".\n";
  //     std::cout << "Slip_min = " << bond_slip.slip_min << ".\n";
  //     std::cout << "Slip_min_old = " << bond_slip.slip_min_old << ".\n";
  //     std::cout << "Slip_max = " << bond_slip.slip_max << ".\n";
  //     std::cout << "Slip_max_old = " << bond_slip.slip_max_old << ".\n";
  //     std::cout << "Bondstress_min = " << bond_slip.bondstress_min << ".\n";
  //     std::cout << "Bondstress_min_old = " << bond_slip.bondstress_min_old << ".\n";
  //     std::cout << "Bondstress_max = " << bond_slip.bondstress_max << ".\n";
  //     std::cout << "Bondstress_max_old = " << bond_slip.bondstress_max_old << ".\n";
  //   }

  const Real slope = 5.0 * _max_bondstress / _transitional_slip[0];
  const Real plastic_slip_max = bond_slip.slip_max_old - bond_slip.bondstress_max_old / slope;
  const Real plastic_slip_min = bond_slip.slip_min_old - bond_slip.bondstress_min_old / slope;

  // if (_debug)
  //   if (_current_node->id() == 6)
  //   {
  //     std::cout << "slope = " << slope << ".\n";
  //     std::cout << "plastic_slip_max = " << plastic_slip_max << ".\n";
  //     std::cout << "plastic_slip_min = " << plastic_slip_min << ".\n";
  //   }

  _bond_stress = 0.0;
  _bond_stress_deriv = 0.0;
  // _bond_stress = _max_bondstress * MathUtils::sign(slip) *
  //                (5.0 * slip_ratio - 4.5 * slip_ratio * slip_ratio +
  //                 1.4 * slip_ratio * slip_ratio * slip_ratio);
  // _bond_stress_deriv = _max_bondstress * (5.0 - 9.0 * slip_ratio + 4.2 * slip_ratio * slip_ratio)
  // /
  //                      _transitional_slip[0];
  if (slip >= bond_slip.slip_max || slip <= bond_slip.slip_min)
  {
    if (slip < _transitional_slip[0] || slip > -_transitional_slip[0])
    {
      if (_debug)
        if (node->id() == 6 || node->id() == 124 || node->id() == 140)
          std::cout << "Calculating bondstress for Case Ia"
                    << ".\n";
      _bond_stress = _max_bondstress * MathUtils::sign(slip) *
                     (5.0 * slip_ratio - 4.5 * slip_ratio * slip_ratio +
                      1.4 * slip_ratio * slip_ratio * slip_ratio);
      _bond_stress_deriv = _max_bondstress *
                           (5.0 - 9.0 * slip_ratio + 4.2 * slip_ratio * slip_ratio) /
                           _transitional_slip[0];
    }
    // }
    else if (slip >= _transitional_slip[0] && slip < _ultimate_slip)
    {
      if (_debug)
        if (node->id() == 6 || node->id() == 124 || node->id() == 140)
          std::cout << "Calculating bondstress for Case Ib"
                    << ".\n";
      _bond_stress = 1.9 * _max_bondstress;
      // _bond_stress_deriv = 1.9 * _max_bondstress;
    }
    else if (slip <= -_transitional_slip[0] && slip > -_ultimate_slip)
    {
      if (_debug)
        if (node->id() == 6 || node->id() == 124 || node->id() == 140)
          std::cout << "Calculating bondstress for Case Ib2"
                    << ".\n";
      _bond_stress = -1.9 * _max_bondstress;
      // _bond_stress_deriv = 1.9 * _max_bondstress;
    }
    else
    {
      if (_debug)
        if (node->id() == 6 || node->id() == 124 || node->id() == 140)
          std::cout << "Calculating bondstress for Case Ic"
                    << ".\n";
      _bond_stress = _frictional_bondstress * MathUtils::sign(slip);
      // _bond_stress_deriv = _frictional_bondstress;
    }
  }
  else if (slip < bond_slip.slip_max && slip > plastic_slip_max)
  {
    if (_debug)
      if (node->id() == 6 || node->id() == 124 || node->id() == 140)
        std::cout << "Calculating bondstress for Case II"
                  << ".\n";

    _bond_stress = (slip - plastic_slip_max) * bond_slip.bondstress_max_old /
                   (bond_slip.slip_max_old - plastic_slip_max);

    _bond_stress_deriv = bond_slip.bondstress_max_old / (bond_slip.slip_max_old - plastic_slip_max);
  }
  else if (slip > bond_slip.slip_min && slip < plastic_slip_min)
  {
    if (_debug)
      if (node->id() == 6 || node->id() == 124 || node->id() == 140)
        std::cout << "Calculating bondstress for Case III"
                  << ".\n";

    _bond_stress = (slip - plastic_slip_min) * bond_slip.bondstress_min_old /
                   (bond_slip.slip_min_old - plastic_slip_min);
    _bond_stress_deriv = bond_slip.bondstress_min_old / (bond_slip.slip_min_old - plastic_slip_min);
  }
  else
    _bond_stress = _frictional_bondstress * MathUtils::sign(slip);

  if (_debug)
  {
    if (node->id() == 6 || node->id() == 124 || node->id() == 140)
    {
      std::cout << "Bondstress = " << _bond_stress << "\n";
      // std::cout << "Bondstress Derivative = " << _bond_stress_deriv << "\n";
    }
  }

  Real bond_force = 2.0 * libMesh::pi * _bar_radius * _current_elem_volume * _bond_stress;
  Real bond_force_deriv =
      2.0 * libMesh::pi * _bar_radius * _current_elem_volume * _bond_stress_deriv;

  RealVectorValue constraint_force_axial = bond_force * _slave_tangent;
  RealVectorValue constraint_force_normal = _penalty * slip_normal;

  _constraint_residual = constraint_force_axial + constraint_force_normal;
  _constraint_jacobian_axial = bond_force_deriv * _slave_tangent;

  // if (_debug)
  // {
  //   if (node->id() == 6)
  //   {
  //     std::cout << "Constraint Residual Axial = " << constraint_force_axial << "\n";
  //     std::cout << "Constraint Residual Normal = " << constraint_force_normal << "\n";
  //     std::cout << "Constraint Residual = " << _constraint_residual << "\n";
  //   }
  // }

  bond_slip.bondstress_min = std::min(bond_slip.bondstress_min_old, _bond_stress);
  bond_slip.bondstress_max = std::max(bond_slip.bondstress_max_old, _bond_stress);

  // if (_debug)
  //   if (_current_node->id() == 6)
  //   {
  //     std::cout << "Bondstress_min = " << bond_slip.bondstress_min << ".\n";
  //     std::cout << "Bondstress_min_old = " << bond_slip.bondstress_min_old << ".\n";
  //     std::cout << "Bondstress_max = " << bond_slip.bondstress_max << ".\n";
  //     std::cout << "Bondstress_max_old = " << bond_slip.bondstress_max_old << ".\n";
  //   }

  _bondslip[node->id()].slip_min = bond_slip.slip_min;
  _bondslip[node->id()].slip_max = bond_slip.slip_max;
  _bondslip[node->id()].bondstress_min = bond_slip.bondstress_min;
  _bondslip[node->id()].bondstress_max = bond_slip.bondstress_max;
}

Real
RebarBondSlipConstraint::computeQpResidual(Moose::ConstraintType type)
{
  Real resid = _constraint_residual(_component);

  switch (type)
  {
    case Moose::Slave:
      return _test_slave[_i][_qp] * resid;

    case Moose::Master:
      return _test_master[_i][_qp] * -resid;
  }

  return 0.0;
}

Real
RebarBondSlipConstraint::computeQpJacobian(Moose::ConstraintJacobianType type)
{
  Real jac_axial = _constraint_jacobian_axial(_component);

  switch (type)
  {
    case Moose::SlaveSlave:
      return _phi_slave[_j][_qp] * jac_axial * _slave_tangent(_component) * _test_slave[_i][_qp] +
             _phi_slave[_j][_qp] * _penalty * _test_slave[_i][_qp] *
                 (1.0 - _slave_tangent(_component) * _slave_tangent(_component));

    case Moose::SlaveMaster:
      return -_phi_master[_j][_qp] * jac_axial * _slave_tangent(_component) * _test_slave[_i][_qp] -
             _phi_master[_j][_qp] * _penalty * _test_slave[_i][_qp] *
                 (1.0 - _slave_tangent(_component) * _slave_tangent(_component));

    case Moose::MasterSlave:
      return -_test_master[_i][_qp] * jac_axial * _slave_tangent(_component) * _phi_slave[_j][_qp] -
             _test_master[_i][_qp] * _penalty * _phi_slave[_j][_qp] *
                 (1.0 - _slave_tangent(_component) * _slave_tangent(_component));

    case Moose::MasterMaster:
      return _test_master[_i][_qp] * jac_axial * _slave_tangent(_component) * _phi_master[_j][_qp] +
             _test_master[_i][_qp] * _penalty * _phi_master[_j][_qp] *
                 (1.0 - _slave_tangent(_component) * _slave_tangent(_component));

    default:
      mooseError("Unsupported type");
      break;
  }
  return 0.0;
}

Real
RebarBondSlipConstraint::computeQpOffDiagJacobian(Moose::ConstraintJacobianType type,
                                                  unsigned int jvar)
{
  // unsigned int coupled_component = std::numeric_limits<unsigned int>::max();
  // bool coupled_var = false;
  // for (unsigned int i = 0; i < _mesh_dimension; ++i)
  // {
  //   if (jvar == _var_nums[i])
  //   {
  //     coupled_component = i;
  //     coupled_var = true;
  //     break;
  //   }
  // }

  unsigned int coupled_component;
  Real axial_component_in_coupled_var_dir = 1.0;
  if (getCoupledVarComponent(jvar, coupled_component))
    axial_component_in_coupled_var_dir = _slave_tangent(coupled_component);

  // std::cout << "Calculating computeOffDiagJacobian for jvar " << jvar << ".\n";
  // if (coupled_var)
  {
    Real jac_axial = _constraint_jacobian_axial(_component);

    switch (type)
    {
      case Moose::SlaveSlave:
        return _phi_slave[_j][_qp] * jac_axial * axial_component_in_coupled_var_dir *
                   _test_slave[_i][_qp] +
               _phi_slave[_j][_qp] * _penalty * _test_slave[_i][_qp] *
                   (-_slave_tangent(_component) * axial_component_in_coupled_var_dir);

      case Moose::SlaveMaster:
        return -_phi_master[_j][_qp] * jac_axial * axial_component_in_coupled_var_dir *
                   _test_slave[_i][_qp] -
               _phi_master[_j][_qp] * _penalty * _test_slave[_i][_qp] *
                   (-_slave_tangent(_component) * axial_component_in_coupled_var_dir);

      case Moose::MasterSlave:
        return -_test_master[_i][_qp] * jac_axial * axial_component_in_coupled_var_dir *
                   _phi_slave[_j][_qp] -
               _test_master[_i][_qp] * _penalty * _phi_slave[_j][_qp] *
                   (-_slave_tangent(_component) * axial_component_in_coupled_var_dir) * 0.0;

      case Moose::MasterMaster:
        return _test_master[_i][_qp] * jac_axial * axial_component_in_coupled_var_dir *
                   _phi_master[_j][_qp] +
               _test_master[_i][_qp] * _penalty * _phi_master[_j][_qp] *
                   (-_slave_tangent(_component) * axial_component_in_coupled_var_dir) * 0.0;

      default:
        mooseError("Unsupported type");
        break;
    }
  }

  return 0.0;
}

void
RebarBondSlipConstraint::getConnectedDofIndices(unsigned int var_num)
{
  unsigned int component;
  if (getCoupledVarComponent(var_num, component))
  {
    // if (_master_slave_jacobian && _connected_slave_nodes_jacobian)
    NodeElemConstraint::getConnectedDofIndices(var_num);
    // else
    // {
    //   _connected_dof_indices.clear();
    //   MooseVariableFEBase & var = _sys.getVariable(0, var_num);
    //   _connected_dof_indices.push_back(var.nodalDofIndex());
    // }
  }

  _phi_slave.resize(_connected_dof_indices.size());

  dof_id_type current_node_var_dof_index = _sys.getVariable(0, var_num).nodalDofIndex();
  _qp = 0;

  // Fill up _phi_slave so that it is 1 when j corresponds to the dof associated with this node
  // and 0 for every other dof
  // This corresponds to evaluating all of the connected shape functions at _this_ node
  for (unsigned int j = 0; j < _connected_dof_indices.size(); j++)
  {
    _phi_slave[j].resize(1);

    if (_connected_dof_indices[j] == current_node_var_dof_index)
      _phi_slave[j][_qp] = 1.0;
    else
      _phi_slave[j][_qp] = 0.0;
  }
}

bool
RebarBondSlipConstraint::getCoupledVarComponent(unsigned int var_num, unsigned int & component)
{
  component = std::numeric_limits<unsigned int>::max();
  bool coupled_var_is_disp_var = false;
  for (unsigned int i = 0; i < LIBMESH_DIM; ++i)
  {
    if (var_num == _var_nums[i])
    {
      coupled_var_is_disp_var = true;
      component = i;
      break;
    }
  }
  return coupled_var_is_disp_var;
}
