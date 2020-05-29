[GlobalParams]
  displacements = 'disp_x disp_y'
[]

[Mesh]
  file = RCBeam_test.e
[]

[Variables]
  [disp_x]
  []
  [disp_y]
  []
[]

# [Functions]
#   [E]
#     type = PiecewiseMultilinear
#     data_file = 'gold/E.txt'
#   []
# []

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
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 500e6
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
  # petsc_options_iname = '-pc_type -ksp_gmres_restart'
  # petsc_options_value = 'jacobi       101'
  petsc_options_iname = '-pc_type'
  petsc_options_value = 'lu'
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

[Modules/TensorMechanics/Master]
  [./Concrete_block]
    block = 1
    # strain = small
    strain = finite
    # incremental = true
   # add_variables = true
    generate_output = 'stress_xx stress_xy stress_yy strain_xx strain_xy strain_yy
    		       max_principal_stress mid_principal_stress min_principal_stress
    		       secondinv_stress thirdinv_stress vonmises_stress
    		       secondinv_strain thirdinv_strain
    		       elastic_strain_xx elastic_strain_xy elastic_strain_yy'
#    		       plastic_strain_xx plastic_strain_xy plas tic_strain_xz plastic_strain_yy plastic_strain_yz plastic_strain_zz'
    save_in = 'resid_x resid_y'
  [../]
[]

[Modules/TensorMechanics/LineElementMaster]
  [./Reinforcement_block]
    block = '2 3'
    truss = true
    area = area
    displacements = 'disp_x disp_y'
    save_in = 'resid_x resid_y'
   # add_variables = true
  [../]
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
    type = RebarBondSlipConstraint
    slave = 2
    master = 1
    penalty = 1e6
    variable = 'disp_x'
    master_variable = 'disp_x'
    component = 0
    max_bondstress = 100.0
    transitional_slip_values = 0.05
    ultimate_slip = 0.1
    # rebar_radius = 1.0
  []
  [rebar_y]
    type = RebarBondSlipConstraint
    slave = 2
    master = 1
    penalty = 1e6
    variable = 'disp_y'
    master_variable = 'disp_y'
    component = 1
    max_bondstress = 100.0
    transitional_slip_values = 0.05
    ultimate_slip = 0.1
  []
[]
