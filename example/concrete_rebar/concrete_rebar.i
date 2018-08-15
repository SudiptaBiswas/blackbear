[GlobalParams]
  displacements = 'disp_x disp_y'
[]

[Mesh]
  file = gold/geo.e
[]

[Variables]
  [disp_x]
  []
  [disp_y]
  []
[]

[Functions]
  [E]
    type = PiecewiseMultilinear
    data_file = 'gold/E.txt'
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
  [area]
    order = CONSTANT
    family = MONOMIAL
  []
  [damage_index]
    order = CONSTANT
    family = MONOMIAL
  []
[]

[AuxKernels]
  [stress_xx]
    type = RankTwoAux
    index_i = 0
    index_j = 0
    variable = stress_xx
    execute_on = 'TIMESTEP_END'
    rank_two_tensor = stress
    block = 1
  []
  [stress_yy]
    type = RankTwoAux
    index_i = 1
    index_j = 1
    variable = stress_yy
    execute_on = 'TIMESTEP_END'
    rank_two_tensor = stress
    block = 1
  []
  [area]
    type = ConstantAux
    block = 2
    variable = area
    value = 7.07e-4
    execute_on = 'initial'
  []
  [damage_index]
    type = MaterialRealAux
    variable = damage_index
    property = damage_index
    block = 1
    execute_on = timestep_end
  []
[]

[Materials]
  [concrete_strain]
    type = ComputeIncrementalSmallStrain
    block = 1
  []
  [Cijkl_concrete]
    type = ComputeIsotropicElasticityTensorFunction
    youngs_modulus = E
    poissons_ratio = 0.2
    block = 1
  []
  [damage]
    type = MazarsDamage
    use_old_damage = true
    tensile_strength = 1e6
    a_t = 0.87
    a_c = 0.65
    b_t = 20000
    b_c = 2150
    block = 1
  []
  [concrete_stress]
    type = ComputeDamageStress
    damage_model = damage
    block = 1
  []
  # [concrete_stress]
  #   type = ComputeFiniteStrainElasticStress
  #   block = 1
  # []
  [truss]
    type = LinearElasticTruss
    block = 2
    youngs_modulus = 2e11
  []
[]

[Executioner]
  type = Transient
  solve_type = PJFNK
  line_search = none
  petsc_options_iname = '-pc_type -ksp_gmres_restart'
  petsc_options_value = 'lu       101'
  nl_rel_tol = 1e-6
  nl_abs_tol = 1e-6
  l_max_its = 100
  nl_max_its = 30
  dt = 2e-5
  end_time = 1
[]

[Outputs]
  exodus = true
  print_linear_residuals = false
[]

[Kernels]
  [solid_x_concrete]
    type = StressDivergenceTensors
    block = 1
    component = 0
    variable = disp_x
    use_displaced_mesh = true
  []
  [solid_y_concrete]
    type = StressDivergenceTensors
    block = 1
    component = 1
    variable = disp_y
    use_displaced_mesh = true
  []
  [solid_x_truss]
    type = StressDivergenceTensorsTruss
    block = 2
    component = 0
    variable = disp_x
    area = area
  []
  [solid_y_truss]
    type = StressDivergenceTensorsTruss
    block = 2
    component = 1
    variable = disp_y
    area = area
  []
[]

[BCs]
  [concrete_top_ydisp]
    type = FunctionDirichletBC
    variable = disp_y
    boundary = '1'
    function = -t
  []
  [concrete_bottom_yfix]
    type = DirichletBC
    variable = disp_y
    boundary = '2 3'
    value = 0
  []
  [concrete_bottom_xfix]
    type = DirichletBC
    variable = disp_x
    boundary = '2'
    value = 0
  []
[]

[Constraints]
  [rebar_x]
    type = EqualValueEmbeddedConstraint
    slave = 2
    master = 1
    penalty = 1e6
    variable = 'disp_x'
    master_variable = 'disp_x'
    formulation = penalty
  []
  [rebar_y]
    type = EqualValueEmbeddedConstraint
    slave = 2
    master = 1
    penalty = 1e6
    variable = 'disp_y'
    master_variable = 'disp_y'
    formulation = penalty
  []
[]
