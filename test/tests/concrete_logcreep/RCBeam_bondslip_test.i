[Mesh]
  file = RC_beam_test.e
[]

[GlobalParams]
  displacements = 'disp_x disp_y'
  # volumetric_locking_correction = true
[]

[Modules/TensorMechanics/Master]
  [./Concrete_block]
    block = 1
    strain = small
    # strain = finite
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

[Variables]
  [./disp_x]
  [../]
  [./disp_y]
  [../]
[]

[AuxVariables]
  [./resid_x]
  [../]
  [./resid_y]
  [../]
  [./area]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./axial_stress]
    order = CONSTANT
    family = MONOMIAL
  [../]
  # [./damage]
  #   order = CONSTANT
  #   family = MONOMIAL
  # [../]
[]

[AuxKernels]
  [./area]
    type = ConstantAux
    block = '2 3'
    variable = area
    value = 2.00e-4 # 509 mm2
    execute_on = 'initial timestep_begin'
  [../]
  [./axial_stress]
    type = MaterialRealAux
    block = '2 3'
    variable = axial_stress
    property = axial_stress
  [../]
  # [damage]
  #   type = MaterialRealAux
  #   block = '1'
  #   property = damage_index
  #   variable = damage
  # []
  # [./area3]
  #   type = ConstantAux
  #   block = 3
  #   variable = area
  #   value = 2.00e-4 # 200 mm2
  #   execute_on = 'initial timestep_begin'
  # [../]
[]

# [Constraints]
#   [./rebar_x2]
#     type = EqualValueEmbeddedConstraint
#     slave = 2
#     master = 1
#     penalty = 1e11
#     variable = 'disp_x'
#     master_variable = 'disp_x'
#     formulation = penalty
#   [../]
#   [./rebar_y2]
#     type = EqualValueEmbeddedConstraint
#     slave = 2
#     master = 1
#     penalty = 1e11
#     variable = 'disp_y'
#     master_variable = 'disp_y'
#     formulation = penalty
#   [../]
#   [./rebar_x3]
#     type = EqualValueEmbeddedConstraint
#     slave = 3
#     master = 1
#     penalty = 1e11
#     variable = 'disp_x'
#     master_variable = 'disp_x'
#     formulation = penalty
#   [../]
#   [./rebar_y3]
#     type = EqualValueEmbeddedConstraint
#     slave = 3
#     master = 1
#     penalty = 1e11
#     variable = 'disp_y'
#     master_variable = 'disp_y'
#     formulation = penalty
#   [../]
# []

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
  [rebar_x2]
    type = RebarBondSlipConstraint
    slave = 3
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
  [rebar_y2]
    type = RebarBondSlipConstraint
    slave = 3
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

[BCs]
  [./loading]
    type = FunctionPresetBC
    variable = disp_x
    boundary = '2'
    function = '1e-4*t'
  [../]
  [./left_support_x]
    type = PresetBC
    variable = disp_x
    boundary = '1'
    value = 0
  [../]
  [./left_support_y]
    type = PresetBC
    variable = disp_y
    # boundary = '100'
    # boundary = '4'
    boundary = '1'
    value = 0
  [../]
[]

[Postprocessors]
  [./deformation_x]
    type = AverageNodalVariableValue
    variable = disp_x
    boundary = '2'
  [../]
  [./deformation_y]
    type = AverageNodalVariableValue
    variable = disp_y
    boundary = '2'
  [../]
  [./react_x]
    type = AverageNodalVariableValue
    variable = resid_x
    boundary = '1'
  [../]
  [./react_y]
    type = AverageNodalVariableValue
    variable = resid_y
    boundary = '1'
  [../]
  [./react_x2]
    type = SideAverageValue
    variable = resid_x
    boundary = '2'
  [../]
  [./react_y2]
    type = SideAverageValue
    variable = resid_y
    boundary = '2'
  [../]
  [./node1_fx]
    type = AverageNodalVariableValue
    variable = resid_x
    boundary = '1001'
    # nodeid = 323
  [../]
  [./node2_fx]
    type = AverageNodalVariableValue
    variable = resid_x
    boundary = '1002'
    # nodeid = 99
  [../]
  [./node2_fy]
    type = NodalVariableValue
    variable = resid_y
    nodeid = 99
  [../]
  [./node1_ux]
    type = AverageNodalVariableValue
    variable = disp_x
    boundary = '1001'
    # nodeid = 323
  [../]
  [./node1_uy]
    type = AverageNodalVariableValue
    variable = disp_y
    boundary = '1002'
    # nodeid = 323
  [../]
  [./node2_ux]
    type = NodalVariableValue
    variable = disp_x
    nodeid = 99
  [../]
  [./node2_uy]
    type = NodalVariableValue
    variable = disp_y
    nodeid = 99
  [../]

  [./stress_xx]
    type = ElementAverageValue
    variable = stress_xx
    block = '1'
  [../]
  [./strain_xx]
    type = ElementAverageValue
    variable = strain_xx
    block = '1'
  [../]
  # [./plastic_strain_truss]
  #   type = ElementAverageValue
  #   variable = plastic_strain
  #   block = '2'
  # [../]
  [./axial_stress]
    type = ElementAverageValue
    variable = axial_stress
    block = '2'
  [../]
  # [damage]
  #   type = ElementAverageValue
  #   variable = damage
  #   block = '1'
  # []
[]

[Materials]
  # Material Properties of concrete
  # [./creep]
  #   type = LinearViscoelasticStressUpdate
  #   block = 1
  # [../]
  # [./logcreep]
  #   type = ConcreteLogarithmicCreepModel
  #   block = 1
  #   poissons_ratio = 0.2
  #   youngs_modulus = 20e9
  #   recoverable_youngs_modulus = 10e9
  #   recoverable_viscosity = 1
  #   long_term_viscosity = 1
  #   long_term_characteristic_time = 1
  # [../]
  # [damage]
  #   type = MazarsDamage
  #   block = 1
  #   tensile_strength = 1e6
  #   a_t = 0.87
  #   a_c = 0.65
  #   b_t = 20000
  #   b_c = 2150
  #   use_old_damage = true
  #   outputs = exodus
  # []
  # [damage]
  #   type = MazarsDamage
  #   use_old_damage = true
  #   tensile_strength = 1e6
  #   a_t = 0.87
  #   a_c = 0.65
  #   b_t = 20000
  #   b_c = 2150
  #   block = 1
  # []
  [concrete_stress]
    type = ComputeLinearElasticStress
    # damage_model = damage
    block = 1
  []
  # [./stress]
  #   type = ComputeMultipleInelasticStress
  #   block = 1
  #   inelastic_models = 'creep'
  #   damage_model = damage
  # [../]
  # Material Properties of steel reinforcement
  # [./truss]
  #   type = LinearElasticTruss
  #   block = '2 3'
  #   youngs_modulus = 2e11
  # [../]
  # [./linelast]
  #   type = PlasticityTruss
  #   block = '2 3'
  #   youngs_modulus = 2.0e11
  #   poissons_ratio = 0.3
  #   hardening_constant = 10000.0
  #   yield_strength = 5e6
  #   # tolerance = 1e-8
  #   displacements = 'disp_x disp_y'
  #   outputs = exodus
  # [../]
  [Cijkl_concrete]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 500e6
    poissons_ratio = 0.2
    block = 1
  []
  [truss]
    type = LinearElasticTruss
    block = '2 3'
    youngs_modulus = 2e11
  []
[]

# [UserObjects]
#   [./visco_update]
#     type = LinearViscoelasticityManager
#     block = 1
#     viscoelastic_model = logcreep
#   [../]
# []

[Preconditioning]
  [./SMP]
    type = SMP
    full = true
  [../]
[]

[Executioner]
  type = Transient
  solve_type = 'NEWTON'
  nl_max_its = 100
  nl_abs_tol = 1.E-5
  nl_rel_tol = 1E-3

  line_search = none

  petsc_options_iname = '-pc_type'
  petsc_options_value = 'lu'

  petsc_options = '-snes_converged_reason'

  end_time = 25
  dtmin = 0.00001

  dt = 0.01
  # [./TimeStepper]
  #   type = LogConstantDT
  #   log_dt = 0.01
  #   first_dt = 0.01
  # [../]
  # type = Transient
  #
  # l_max_its  = 50
  # l_tol      = 1e-8
  # nl_max_its = 20
  # nl_rel_tol = 1e-10
  # nl_abs_tol = 1e-8
  #
  # dtmin = 0.01
  # end_time = 100
  # [./TimeStepper]
  #   type = LogConstantDT
  #   log_dt = 0.1
  #   first_dt = 0.1
  # [../]
[]


[Outputs]
  # print_linear_residuals = false
  exodus = true
  csv = true
  # file_base = RCBeam_plastic_tens_damage
[]
