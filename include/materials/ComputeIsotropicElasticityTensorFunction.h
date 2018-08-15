/****************************************************************/
/* MOOSE - Multiphysics Object Oriented Simulation Environment  */
/*                                                              */
/*          All contents are licensed under LGPL V2.1           */
/*             See LICENSE for full restrictions                */
/****************************************************************/
#ifndef ComputeIsotropicElasticityTensorFunction_H
#define ComputeIsotropicElasticityTensorFunction_H

#include "ComputeElasticityTensorBase.h"
#include "Function.h"

class ComputeIsotropicElasticityTensorFunction;
template <>
InputParameters validParams<ComputeIsotropicElasticityTensorFunction>();

/**
 * ComputeIsotropicElasticityTensorFunction defines an elasticity tensor material for
 * isotropic materials.
 */
class ComputeIsotropicElasticityTensorFunction : public ComputeElasticityTensorBase
{
public:
  ComputeIsotropicElasticityTensorFunction(const InputParameters & parameters);

protected:
  virtual void computeQpElasticityTensor() override;

  Function & _E;
  Function & _nu;
};

#endif // ComputeIsotropicElasticityTensorFunction_H
