[Mesh]
  [./cont]
    type = GeneratedMeshGenerator
    dim = 2
    nx = 10
    ny = 10
  [../]
  [./cont_sidesets]
    type = RenameBoundaryGenerator
    input = cont
    old_boundary_id = '0 1 2 3'
    new_boundary_name = 'cont_bottom cont_right cont_top cont_left'
  [../]
  [./cont_id]
    type = SubdomainIDGenerator
    input = cont_sidesets
    subdomain_id = 0
  [../]
  [./truss]
    type = GeneratedMeshGenerator
    dim = 1
    elem_type = EDGE
    nx = 5
    # xmin = -1
 [../]
 # [./truss_sidesets]
 #   type = RenameBoundaryGenerator
 #   input = truss
 #   old_boundary_id = '0 1'
 #   new_boundary_name = 'truss_left truss_right'
 # [../]
 [./truss_id]
   type = SubdomainIDGenerator
   input = truss
   subdomain_id = 1
 [../]

 [./combined]
   type = MeshCollectionGenerator
   inputs = 'cont_id truss_id'
 [../]

[]

[GlobalParams]
  displacements = 'disp_x disp_y'
  block = '1 0'
[]

[Variables]
  [./disp_x]
    order = FIRST
    family = LAGRANGE
  [../]
  [./disp_y]
    order = FIRST
    family = LAGRANGE
  [../]
[]

[AuxVariables]
 [./axial_stress]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./e_over_l]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./area]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./react_x]
    order = FIRST
    family = LAGRANGE
  [../]
  [./hardening]
     order = CONSTANT
     family = MONOMIAL
   [../]
[]

[Modules/TensorMechanics/Master]
  [./Concrete_block]
    block = 0
    strain = SMALL
    incremental = true
  [../]
[]

[Functions]
  [./hf]
    type = PiecewiseLinear
    x = '0    0.0001  0.0003  0.0023'
    y = '50e6 52e6    54e6    56e6'
  [../]
[]

[BCs]
  [./fixx]
    type = DirichletBC
    variable = disp_x
    boundary = 'cont_left'
    value = 0.0
  [../]
  [./fixy]
    type = DirichletBC
    variable = disp_y
    boundary = cont_left
    value = 0.0
  [../]
  [./load]
    type = FunctionPresetBC
    variable = disp_x
    boundary = cont_right
    function = 't'
  [../]
  [./fixx_truss]
    type = DirichletBC
    variable = disp_x
    boundary = 0
    value = 0.0
  [../]
  [./fixy_truss]
    type = DirichletBC
    variable = disp_y
    boundary = 0
    value = 0.0
  [../]
  # [./load]
  #   type = FunctionPresetBC
  #   variable = disp_x
  #   boundary = 1
  #   function = 't'
  # [../]
[]

[AuxKernels]
  # [./axial_stress]
  #   type = MaterialRealAux
  #   property = axial_stress
  #   variable = axial_stress
  #   block = 1
  # [../]
  # [./e_over_l]
  #   type = MaterialRealAux
  #   property = e_over_l
  #   variable = e_over_l
  #   block = 1
  # [../]
  # [./area]
  #   type = ConstantAux
  #   variable = area
  #   value = 1.0
  #   execute_on = 'initial timestep_begin'
  #   block = 1
  # [../]
  [./hardening]
    type = MaterialRealAux
    property = hardening_variable
    variable = hardening
  [../]
[]

[Postprocessors]
  # [./s_xx]
  #   type = ElementIntegralMaterialProperty
  #   mat_prop = axial_stress
  #   block = 1
  # [../]
  # [./e_xx]
  #   type = ElementIntegralMaterialProperty
  #   mat_prop = total_stretch
  #   block = 1
  # [../]
  # [./ee_xx]
  #   type = ElementIntegralMaterialProperty
  #   mat_prop = elastic_stretch
  #   block = 1
  # [../]
  # [./ep_xx]
  #   type = ElementIntegralMaterialProperty
  #   mat_prop = plastic_stretch
  #   block = 1
  # [../]
[]

[Executioner]
  type = Transient
  solve_type = 'PJFNK'
  petsc_options = '-snes_ksp_ew'
  petsc_options_iname = '-pc_type'
  petsc_options_value = 'lu'
  nl_abs_tol = 1e-11
  l_max_its = 20
  dt = 5e-5
  num_steps = 10
[]

[Kernels]
  [./solid]
    type = StressDivergenceTensorsTruss
    component = 0
    variable = disp_x
    area = area
    # save_in = react_x
    block = 1
  [../]
  [./solidy]
    type = StressDivergenceTensorsTruss
    component = 0
    variable = disp_y
    area = area
    # save_in = react_x
    block = 1
  [../]
[]

[Materials]
  [./truss]
    type = PlasticTruss
    youngs_modulus = 2.0e11
    yield_stress = 500e5
    hardening_constant = 0
    block = '1'
    outputs = exodus
  [../]
  [./elasticity_tensor]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 210666666666.666667
    poissons_ratio = 0.3333333333333333
    block = '0'
  [../]
  [./isotropic_plasticity]
    type = IsotropicPlasticityStressUpdate
    yield_stress = 285788383.2488647 # = sqrt(3)*165e6 = sqrt(3) * yield in shear
    hardening_constant = 0.0
    block = '0'
  [../]
  [./radial_return_stress]
    type = ComputeMultipleInelasticStress
    tangent_operator = elastic
    inelastic_models = 'isotropic_plasticity'
    block = '0'
  [../]
[]

[Outputs]
  exodus = true
  csv = true
[]
