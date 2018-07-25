[GlobalParams]
  displacements = 'disp_x disp_y'
[]

[Mesh]
  file = gold/patch.e
  uniform_refine = 0
[]

[Variables]
  [disp_x]
  []
  [disp_y]
  []
[]

[AuxVariables]
  [stress_xx]
    order = CONSTANT
    family = MONOMIAL
  []
  [stress_yy]
    order = CONSTANT
    family = MONOMIAL
  []
  [./area]
    order = CONSTANT
    family = MONOMIAL
  [../]
[]

[AuxKernels]
  [stress_xx]
    type = MaterialRankTwoTensorAux
    i = 0
    j = 0
    variable = stress_xx
    execute_on = 'TIMESTEP_END'
    property = stress
    block = 1
  []
  [stress_yy]
    type = MaterialRankTwoTensorAux
    i = 1
    j = 1
    variable = stress_yy
    execute_on = 'TIMESTEP_END'
    property = stress
    block = 1
  []
  [./area]
    type = ConstantAux
    block = 2
    variable = area
    value = 0.05
    execute_on = 'initial timestep_begin'
  [../]
[]

[Materials]
  [Cijkl_concrete]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 1e6
    poissons_ratio = 0
    block = 1
  []
  [strain_concrete]
    type = ComputeSmallStrain
    block = 1
  []
  [stress_concrete]
    type = ComputeLinearElasticStress
    block = 1
  []

  [./truss]
    type = LinearElasticTruss
    block = 2
    youngs_modulus = 1e6
  [../]
[]

[Executioner]
  type = Transient
  solve_type = PJFNK
  petsc_options_iname = '-pc_type -ksp_gmres_restart'
  petsc_options_value = 'lu       101'
  line_search = none
  nl_rel_tol = 1e-15
  nl_abs_tol = 1e-8
  l_max_its = 100
  nl_max_its = 10
  dt = 0.01
  end_time = 0.2
  # num_steps = 10
[]

[Outputs]
  exodus = true
  print_linear_residuals = false
[]

[Kernels]
  [TensorMechanics]
    block = '1'
  []
  [./solid_x]
    type = StressDivergenceTensorsTruss
    block = 2
    component = 0
    variable = disp_x
    area = area
  [../]
  [./solid_y]
    type = StressDivergenceTensorsTruss
    block = 2
    component = 1
    variable = disp_y
    area = area
  [../]
[]

[BCs]
  [concrete_top_ydisp]
    type = FunctionDirichletBC
    variable = disp_y
    boundary = 4
    function = t
  []
  [concrete_bottom_yfix]
    type = DirichletBC
    variable = disp_y
    boundary = 2
    value = 0
  []
  [concrete_bottom_xfix]
    type = DirichletBC
    variable = disp_x
    boundary = 2
    value = 0
  []
[]

[Constraints]
  [rebar_x]
    type = ConcreteRebarConstraint
    slave = 2
    master = 1
    penalty = 1e6
    variable = 'disp_x'
    master_variable = 'disp_x'
    displacements = 'disp_x disp_y'
    component = 0
    formulation = penalty
    model = glued
  []
  [rebar_y]
    type = ConcreteRebarConstraint
    slave = 2
    master = 1
    penalty = 1e6
    variable = 'disp_y'
    master_variable = 'disp_y'
    displacements = 'disp_x disp_y'
    component = 1
    formulation = penalty
    model = glued
  []
[]
