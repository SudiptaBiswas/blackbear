[GlobalParams]
  displacements = 'disp_x disp_y disp_z'
  # volumetric_locking_correction = true
[]

[Mesh]
  file = Block_ASR_Uniax3L.e

  # [cont]
  #   type = FileMeshGenerator
  #   file = Block_ASR_Uniax3L.e
  # []
  # [./cont_sidesets]
  #   type = RenameBoundaryGenerator
  #   input = cont
  #   old_boundary_id = '101 102 103 104 105 106'
  #   new_boundary_name = 'cont_front cont_back cont_bottom cont_left cont_top cont_right '
  # [../]

  # [./cont_sidesets]
  #   type = RenameBoundaryGenerator
  #   input = cont
  #   old_boundary_id = '1 2 3 4 5 6'
  #   new_boundary_name = 'cont_front cont_back cont_bottom cont_left cont_top cont_right '
  # [../]
[]

[AuxVariables]
  [./resid_x]
  [../]
  [./resid_y]
  [../]
  [./resid_z]
  [../]
  [./T]
    order = FIRST
    family = LAGRANGE
    initial_condition = 20.0
  [../]

  [./ASR_ex]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]

  [./ASR_vstrain]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./ASR_strain_xx]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./ASR_strain_yy]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./ASR_strain_zz]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./ASR_strain_xy]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./ASR_strain_yz]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./ASR_strain_zx]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./volumetric_strain]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./area]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./axial_stress]
    order = CONSTANT
    family = MONOMIAL
  [../]
[]

[Modules/TensorMechanics/Master]
  [./concrete]
    block = 1
    # strain = FINITE
    strain = SMALL
    add_variables = true
    eigenstrain_names = 'asr_expansion'
    generate_output = 'stress_xx stress_yy stress_zz stress_xy stress_yz stress_zx vonmises_stress hydrostatic_stress elastic_strain_xx elastic_strain_yy elastic_strain_zz strain_xx strain_yy strain_zz '
    save_in = 'resid_x resid_y resid_z'
  [../]
[]

[Modules/TensorMechanics/LineElementMaster]
  [./Reinforcement_block]
    block = '2 '
    truss = true
    area = area
    displacements = 'disp_x disp_y disp_z'
    save_in = 'resid_x resid_y resid_z'
   # add_variables = true
  [../]
[]

[Constraints]
  [./rebar_x2]
    type = EqualValueEmbeddedConstraint
    slave = 2
    master = 1
    penalty = 31e6
    variable = 'disp_x'
    master_variable = 'disp_x'
    formulation = penalty
  [../]
  [./rebar_y2]
    type = EqualValueEmbeddedConstraint
    slave = 2
    master = 1
    penalty = 31e6
    variable = 'disp_y'
    master_variable = 'disp_y'
    formulation = penalty
  [../]
  [./rebar_z2]
    type = EqualValueEmbeddedConstraint
    slave = 2
    master = 1
    penalty = 31e6
    variable = 'disp_z'
    master_variable = 'disp_z'
    formulation = penalty
  [../]
[]


[AuxKernels]
  [./ASR_ex]
    type = MaterialRealAux
    variable = ASR_ex
    block = 1
    property = ASR_extent
    execute_on = 'timestep_end'
  [../]
  [./ASR_vstrain]
    type = MaterialRealAux
    block = 1
    variable = ASR_vstrain
    property = ASR_volumetric_strain
    execute_on = 'timestep_end'
  [../]

  [./ASR_strain_xx]
    type = RankTwoAux
    block = 1
    rank_two_tensor = asr_expansion
    variable = ASR_strain_xx
    index_i = 0
    index_j = 0
    execute_on = 'timestep_end'
  [../]
  [./ASR_strain_yy]
    type = RankTwoAux
    block = 1
    rank_two_tensor = asr_expansion
    variable = ASR_strain_yy
    index_i = 1
    index_j = 1
    execute_on = 'timestep_end'
  [../]
  [./ASR_strain_zz]
    type = RankTwoAux
    block = 1
    rank_two_tensor = asr_expansion
    variable = ASR_strain_zz
    index_i = 2
    index_j = 2
    execute_on = 'timestep_end'
  [../]

  [./ASR_strain_xy]
    type = RankTwoAux
    block = 1
    rank_two_tensor = asr_expansion
    variable = ASR_strain_xy
    index_i = 0
    index_j = 1
    execute_on = 'timestep_end'
  [../]

  [./ASR_strain_yz]
    type = RankTwoAux
    block = 1
    rank_two_tensor = asr_expansion
    variable = ASR_strain_yz
    index_i = 1
    index_j = 2
    execute_on = 'timestep_end'
  [../]

  [./ASR_strain_zx]
    type = RankTwoAux
    block = 1
    rank_two_tensor = asr_expansion
    variable = ASR_strain_zx
    index_i = 0
    index_j = 2
    execute_on = 'timestep_end'
  [../]

  [./volumetric_strain]
    type = RankTwoScalarAux
    scalar_type = VolumetricStrain
    rank_two_tensor = total_strain
    variable = volumetric_strain
    block = 1
  [../]

  [./area]
    type = ConstantAux
    block = '2'
    variable = area
    value = 2.00e-4 # 509 mm2
    execute_on = 'initial timestep_begin'
  [../]
  [./axial_stress]
    type = MaterialRealAux
    block = '2'
    variable = axial_stress
    property = axial_stress
  [../]
[]

[Functions]
  [./strain_function]
    type = ParsedFunction
    value = 0.6e-2*(1-exp(-t/86400/35))/(1+exp((35-t/86400)/50))
  [../]
[]

[Materials]
  [elasticity_concrete]
    type = ComputeIsotropicElasticityTensor
    block = 1
    youngs_modulus = 37.3e9
    poissons_ratio = 0.22
  []
  [stress]
    type = ComputeLinearElasticStress
    # type = ComputeFiniteStrainElasticStress
    block = 1
  []

  [ASR_expansion]
    type = ConcreteASREigenstrain
    block = 1
    expansion_type = Isotropic

    reference_temperature  = 20.0
    temperature_unit = Celsius
    max_volumetric_expansion =  0.6e-2

    characteristic_time = 35
    latency_time = 50
    characteristic_activation_energy = 5400.0
    latency_activation_energy = 9400.0
    stress_latency_factor = 1.0

    compressive_strength = 31.0e6
    compressive_stress_exponent = 0.0

    tensile_strength = 3.2e6
    tensile_retention_factor = 1.0
    tensile_absorption_factor = 1.0

    ASR_dependent_tensile_strength = false
    residual_tensile_strength_fraction = 1.0

    temperature = T
    relative_humidity = 0.0
    rh_exponent = 0.0
    eigenstrain_name = asr_expansion
    absolute_tolerance = 1e-10
    output_iteration_info_on_error = true
  []
  [truss]
    type = LinearElasticTruss
    block = '2 '
    youngs_modulus = 2e11
  []
[]


[BCs]
  [./left]
    type = DirichletBC
    variable = disp_x
    # boundary = cont_left
    boundary = 104
    value = 0.0
  [../]
  [./bottom]
    type = DirichletBC
    variable = disp_y
    # boundary = cont_bottom
    boundary = 103
    value = 0.0
  [../]
  [./back]
    type = DirichletBC
    variable = disp_z
    # boundary = cont_back
    boundary = 102
    value = 0.0
  [../]
  [./barx]
    type = DirichletBC
    variable = disp_x
    boundary = 1001
    value = 0.0
  [../]
  [./bary]
    type = DirichletBC
    variable = disp_y
    boundary = 1001
    value = 0.0
  [../]
  [./barz]
    type = DirichletBC
    variable = disp_z
    boundary = 1001
    value = 0.0
  [../]
[]

[Postprocessors]
  [./ASR_strain]
    type = ElementAverageValue
    variable = ASR_vstrain
    block = 1
  [../]
  [ASR_ext]
    type = ElementAverageValue
    variable = ASR_ex
    block = 1
  []
  [./vonmises]
    type = ElementAverageValue
    variable = vonmises_stress
    block = 1
  [../]
  [./vstrain]
    type = ElementAverageValue
    variable = volumetric_strain
    block = 1
  [../]

  [./error_ASR]
    type = ElementL2Error
    variable = ASR_vstrain
    block = 1
    function=strain_function
  [../]
  [./error]
    type = ElementL2Error
    variable = volumetric_strain
    block = 1
    function=strain_function
  [../]
[]

[Executioner]
  type       = Transient
  solve_type = 'PJFNK'
  line_search = none
  # petsc_options_iname = '-pc_type -ksp_gmres_restart'
  # petsc_options_value = 'lu       101'

  petsc_options_iname = '-pc_type -pc_hypre_type -ksp_gmres_restart -snes_ls -pc_hypre_boomeramg_strong_threshold'
  petsc_options_value = 'hypre boomeramg 201 cubic 0.7'

  dt = 100000
  end_time = 38880000
  # end_time = 12960000
  l_max_its  = 50
  l_tol      = 1e-4
  nl_max_its = 10
  nl_rel_tol = 1e-8
  nl_abs_tol = 1e-10
[]

[Outputs]
  exodus         = true
  perf_graph     = true
  csv = true
  [./Console]
    type = Console
  [../]
[]

[Debug]
  show_var_residual_norms = true
[]
