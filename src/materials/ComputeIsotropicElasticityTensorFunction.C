/****************************************************************/
/* MOOSE - Multiphysics Object Oriented Simulation Environment  */
/*                                                              */
/*          All contents are licensed under LGPL V2.1           */
/*             See LICENSE for full restrictions                */
/****************************************************************/
#include "ComputeIsotropicElasticityTensorFunction.h"

registerMooseObject("BlackBearApp", ComputeIsotropicElasticityTensorFunction);

template <>
InputParameters
validParams<ComputeIsotropicElasticityTensorFunction>()
{
  InputParameters params = validParams<ComputeElasticityTensorBase>();
  params.addClassDescription("Compute a constant isotropic elasticity tensor.");
  params.addRequiredParam<FunctionName>("youngs_modulus", "Young's modulus for the material.");
  params.addRequiredParam<FunctionName>("poissons_ratio", "Poisson's ratio of the material.");
  return params;
}

ComputeIsotropicElasticityTensorFunction::ComputeIsotropicElasticityTensorFunction(
    const InputParameters & parameters)
  : ComputeElasticityTensorBase(parameters),
    _E(getFunction("youngs_modulus")),
    _nu(getFunction("poissons_ratio"))
{
  // all tensors created by this class are always isotropic
  issueGuarantee(_elasticity_tensor_name, Guarantee::ISOTROPIC);
}

void
ComputeIsotropicElasticityTensorFunction::computeQpElasticityTensor()
{
  std::vector<Real> iso_const(2);
  iso_const[0] = _E.value(_t, _q_point[_qp]);
  iso_const[1] = _nu.value(_t, _q_point[_qp]);
  _elasticity_tensor[_qp].fillFromInputVector(iso_const, RankFourTensor::symmetric_isotropic_E_nu);
}
