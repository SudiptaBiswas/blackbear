//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html

#ifndef CONCRETEREBARCONSTRAINT_H
#define CONCRETEREBARCONSTRAINT_H

// MOOSE includes
#include "EqualValueEmbeddedConstraint.h"

/// Models, currently only supports GLUED and BONDSLIP
enum Model
{
  GLUED,
  BONDSLIP,
  MODEL_INVALID
};

// Forward Declarations
class ConcreteRebarConstraint;

template <>
InputParameters validParams<ConcreteRebarConstraint>();

/// A ConcreteRebarConstraint enforces concrete-rebar constraint
class ConcreteRebarConstraint : public EqualValueEmbeddedConstraint
{
public:
  ConcreteRebarConstraint(const InputParameters & parameters);

  /**
   * Compute the reaction force required to enforce the constraint based on the specified
   * formulation. The constraint force is computed once per constraint
   */
  void computeConstraintForce() override;

protected:
  virtual Real computeQpResidual(Moose::ConstraintType type) override;
  virtual Real computeQpJacobian(Moose::ConstraintJacobianType type) override;
  virtual Real computeQpOffDiagJacobian(Moose::ConstraintJacobianType type,
                                        unsigned int jvar) override;
  const unsigned _component;
  const unsigned int _mesh_dimension;
  std::vector<unsigned int> _var_nums;
  std::vector<MooseVariable *> _vars;
  /**
   * Get the Formulation enum from a case-insensitive string
   * @param name name of the formulation string to be parsed
   * @return Formulation parsed enum of the corresponding formulation, returns INVALID by default
   */
  static Model getModel(std::string name);
  /// Enum used to define the formulation used to impose the constraint
  const Model _model;
  /// constraint force needed to enforce the constraint
  RealVectorValue _constraint_force;
  /// penalty force for the current constraint
  RealVectorValue _pen_force;
};

#endif
