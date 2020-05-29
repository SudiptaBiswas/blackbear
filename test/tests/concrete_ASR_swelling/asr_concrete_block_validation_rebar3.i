[GlobalParams]
  displacements = 'disp_x disp_y disp_z'
  # volumetric_locking_correction = true
[]

[Mesh]
  # file = concrete_block_mesh.e
  file = ASR_block_unixyz3l.e

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

[Variables]
  [./T]
    order = FIRST
    family = LAGRANGE
    initial_condition = 35.0
  [../]
  [./rh]
    order = FIRST
    family = LAGRANGE
    initial_condition = 0.2
  [../]
[]

[AuxVariables]
  [./resid_x]
  [../]
  [./resid_y]
  [../]
  [./resid_z]
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
  [./thermal_strain_xx]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./thermal_strain_yy]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./thermal_strain_zz]
    order = CONSTANT
    family = MONOMIAL
    block = 1
  [../]
  [./thermal_conductivity]
    order = CONSTANT
    family = Monomial
  [../]

  [./thermal_capacity]
    order = CONSTANT
    family = Monomial
  [../]

  [./moisture_capacity]
    order = CONSTANT
    family = Monomial
  [../]

  [./humidity_diffusivity]
    order = CONSTANT
    family = Monomial
  [../]

  [./water_content]
    order = CONSTANT
    family = Monomial
  [../]
  [./water_hydrated]
    order = CONSTANT
    family = Monomial
  [../]
  [damage_index]
    order = CONSTANT
    family = MONOMIAL
  []

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
    # strain = SMALL
    strain = FINITE
    add_variables = true
    eigenstrain_names = 'asr_expansion thermal_expansion'
    generate_output = 'stress_xx stress_yy stress_zz stress_xy stress_yz stress_zx vonmises_stress hydrostatic_stress elastic_strain_xx elastic_strain_yy elastic_strain_zz strain_xx strain_yy strain_zz'
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


[Kernels]
  [./T_td]
    type     = ConcreteThermalTimeIntegration
    variable = T
    block = 1
  [../]
  [./T_diff]
    type     = ConcreteThermalConduction
    variable = T
    block = 1
  [../]

  [./T_conv]
    type     = ConcreteThermalConvection
    variable = T
    relative_humidity = rh
    block = 1
  [../]

  [./T_adsorption]
    type     = ConcreteLatentHeat
    variable = T
    H = rh
    block = 1
  [../]

  [./rh_td]
    type     = ConcreteMoistureTimeIntegration
    variable = rh
    block = 1
  [../]

  [./rh_diff]
    type     = ConcreteMoistureDiffusion
    variable = rh
    temperature = T
    block = 1
  [../]
  [./heat_dt]
    type = TimeDerivative
    variable = T
    block = 2
  [../]
  [./heat_conduction]
    type = HeatConduction
    variable = T
    diffusion_coefficient = 53.0
    block = 2
  [../]

  # [./rh_dehydration]
  #  type     = ConcreteMoistureDehydration
  #  variable = rh
  #  temperature = T
  # [../]
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
  [./thermal_strain_xx]
    type = RankTwoAux
    block = 1
    rank_two_tensor = thermal_expansion
    variable = thermal_strain_xx
    index_i = 0
    index_j = 0
    execute_on = 'timestep_end'
  [../]
  [./thermal_strain_yy]
    type = RankTwoAux
    block = 1
    rank_two_tensor = thermal_expansion
    variable = thermal_strain_yy
    index_i = 1
    index_j = 1
    execute_on = 'timestep_end'
  [../]
  [./thermal_strain_zz]
    type = RankTwoAux
    block = 1
    rank_two_tensor = thermal_expansion
    variable = thermal_strain_zz
    index_i = 2
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

  [./k]
    type = MaterialRealAux
    variable = thermal_conductivity
    property = thermal_conductivity
    execute_on = 'timestep_end'
    block = 1
  [../]
  [./capacity]
    type = MaterialRealAux
    variable = thermal_capacity
    property = thermal_capacity
    execute_on = 'timestep_end'
    block = 1
  [../]

  [./rh_capacity]
    type = MaterialRealAux
    variable = moisture_capacity
    property = moisture_capacity
    execute_on = 'timestep_end'
    block = 1
  [../]
  [./rh_duff]
    type = MaterialRealAux
    variable = humidity_diffusivity
    property = humidity_diffusivity
    execute_on = 'timestep_end'
    block = 1
  [../]
  [./wc_duff]
    type = MaterialRealAux
    variable = water_content
    property = moisture_content
    execute_on = 'timestep_end'
    block = 1
  [../]
  [./hydrw_duff]
    type = MaterialRealAux
    variable = water_hydrated
    property = hydrated_water
    execute_on = 'timestep_end'
    block = 1
  [../]
  [./area]
    type = ConstantAux
    block = '2'
    variable = area
    # value = 3.8e-4 # 1.33e-4
    value = 1.33e-4
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

  [./ramp1]
    type = PiecewiseLinear
    x = '0.0  2592000.0 5184000 7776000 10368000 12960000 15552000 18144000 20736000 23328000 25920000 28512000 31104000 33696000 36288000 38880000'
    # y = '20.0  30.0      33.0     37.0   40.0     43.0    45.0      46.0     42.0     38.0     32.0     20.0     30.0      33.0     37.0   40.0    '
    # y = '  40.0  43.0    45.0    46.0     42.0     38.0     32.0     20.0     20.0     30.0    33.0     37.0      40.0    43.0    45.0       46.0'
    # y = ' 40.0  43.0    45.0    46.0     42.0     38.0     32.0     20.0     20.0      30.0    33.0     37.0      42.0    45.0     48.0       50.0'
    y =  ' 46.0  42.0     38.0     32.0     20.0     22.0      30.0    35.0     38.0      43.0     46.0     48.0     50.0      48.0      46.0     45.0   '
    #scale_factor = 0.2  2592000
  [../]

  [./ramp2]
    type = PiecewiseConstant
    x = '604800.0  1209600 1814400  2419200 3024000 3628800 4233600 4838400   5443200  6048000 6652800 7257600 7862400 8467200 9072000 9676800 10281600 10886400 11491200 12096000 12700800 13305600 13910400 14515200 15120000 15724800 16329600 16934400 17539200 18144000 18748800 19353600 19958400 20563200 21168000 21772800 22377600 22982400 23587200 24192000 24796800 25401600 26006400
    26611200 27216000 27820800 28425600 29030400 29635200 30240000 30844800 31449600 32054400 32659200 33264000 33868800 34473600 35078400 35683200 36288000 36892800 37497600 38102400 38707200  38880000'
    y = '0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8   0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8
    0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8 0.2  0.8  0.2'
    direction = left
  [../]
[]

[Materials]

  [./concrete]
    type = PorousMediaBase
    block = 1
    # setup thermal property models and parameters
    # options available: CONSTANT ASCE-1992 KODUR-2004 EUROCODE-2004 KIM-2003
    thermal_conductivity_model =  KODUR-2004
    thermal_capacity_model     =  KODUR-2004
    aggregate_type = Siliceous               #options: Siliceous Carbonate

    ref_density_of_concrete = 2231.0         # in kg/m^3
    ref_specific_heat_of_concrete = 1100.0   # in J/(Kg.0C)
    ref_thermal_conductivity_of_concrete = 3 # in W/(m.0C)


    # setup moisture capacity and humidity diffusivity models
    aggregate_pore_type = dense              #options: dense porous
    aggregate_mass = 1877.0                  #mass of aggregate (kg) per m^3 of concrete
    cement_type = 1                          #options: 1 2 3 4
    cement_mass = 354.0                      #mass of cement (kg) per m^3 of concrete
    water_to_cement_ratio       = 0.5
    concrete_cure_time          = 15.0       #curing time in (days)

    # options available for humidity diffusivity:
    moisture_diffusivity_model = Bazant      #options: Bazant Xi Mensi
    D1 = 3.0e-10
    aggregate_vol_fraction = 0.7             #used in Xi's moisture diffusivity model

    coupled_moisture_diffusivity_factor = 1.0e-3  # factor for mositure diffusivity due to heat

    # coupled nonlinear variables
    relative_humidity = rh
    temperature = T
  [../]

  [elasticity_concrete]
    type = ComputeIsotropicElasticityTensor
    block = 1
    youngs_modulus = 37.3e9
    poissons_ratio = 0.22
  []
  # [stress]
  #   type = ComputeLinearElasticStress
  #   # type = ComputeFiniteStrainElasticStress
  #   block = 1
  # []
  [ASR_damage_concrete]
    type = ConcreteASRMicrocrackingDamage
    block = 1
    residual_youngs_modulus_fraction = 0.5
  []

  [stress_concrete]
    type = ComputeDamageStress
    block = 1
    damage_model = ASR_damage_concrete
  []

  [ASR_expansion]
    type = ConcreteASREigenstrain
    block = 1
    expansion_type = Anisotropic

    reference_temperature  = 35.0
    temperature_unit = Celsius
    max_volumetric_expansion = 1.72e-2

    characteristic_time = 20.9
    latency_time = 148.6
    characteristic_activation_energy = 5400.0
    latency_activation_energy = 9400.0
    stress_latency_factor = 1.0

    compressive_strength = 31.0e6
    compressive_stress_exponent = 0.0
    expansion_stress_limit = 8.0e6

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

  [thermal_strain_concrete]
    type = ComputeThermalExpansionEigenstrain
    block = 1
    temperature = T
    thermal_expansion_coeff = 8.0e-6
    stress_free_temperature = 35.0
    eigenstrain_name = thermal_expansion
  []

  [truss]
    type = LinearElasticTruss
    block = '2 '
    youngs_modulus = 2e11
    temperature = T
    thermal_expansion_coeff = 11.3e-6
    temperature_ref = 35.0
  []

[]


[BCs]
  [./left]
    type = DirichletBC
    variable = disp_x
    # boundary = cont_left
    boundary = '2000 2005'
    value = 0.0
  [../]
  [./bottom]
    type = DirichletBC
    variable = disp_y
    # boundary = cont_bottom
    boundary = '2000 2001'
    value = 0.0
  [../]
  [./back]
    type = DirichletBC
    variable = disp_z
    # boundary = cont_back
    boundary = '2000 2005'
    value = 0.0
  [../]
  [./T]
    type = FunctionDirichletBC
    variable = T
    boundary = '101 102 103 104 105 106'
    function = ramp1
  [../]
  [./rh]
    type = FunctionDirichletBC
    # type = DirichletBC
    variable = rh
    boundary = '101 102 103 104 105 106'
    # value = 0.2
    function = ramp2
  [../]
  #
  # [./barx]
  #   type = DirichletBC
  #   variable = disp_x
  #   boundary = '1001 1002'
  #   value = 0.0
  # [../]
  # [./bary]
  #   type = DirichletBC
  #   variable = disp_y
  #   boundary = '1001 1002'
  #   value = 0.0
  # [../]
  # [./barz]
  #   type = DirichletBC
  #   variable = disp_z
  #   boundary = '1001 1002'
  #   value = 0.0
  # [../]
[]

[Postprocessors]
  [./ASR_strain]
    type = ElementAverageValue
    variable = ASR_vstrain
    block = 1
  [../]
  [./ASR_strain_xx]
    type = ElementAverageValue
    variable = ASR_strain_xx
    block = 1
  [../]
  [./ASR_strain_yy]
    type = ElementAverageValue
    variable = ASR_strain_yy
    block = 1
  [../]
  [./ASR_strain_zz]
    type = ElementAverageValue
    variable = ASR_strain_zz
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
  [./strain_xx]
    type = ElementAverageValue
    variable = strain_xx
    block = 1
  [../]
  [./strain_yy]
    type = ElementAverageValue
    variable = strain_yy
    block = 1
  [../]
  [./strain_zz]
    type = ElementAverageValue
    variable = strain_zz
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
  [./temp]
    type = ElementAverageValue
    variable = T
    block = 1
  [../]
  [./humidity]
    type = ElementAverageValue
    variable = rh
    block = 1
  [../]
  [./thermal_strain_xx]
    type = ElementAverageValue
    variable = thermal_strain_xx
    block = 1
  [../]
  [./thermal_strain_yy]
    type = ElementAverageValue
    variable = thermal_strain_yy
    block = 1
  [../]
  [./thermal_strain_zz]
    type = ElementAverageValue
    variable = thermal_strain_zz
    block = 1
  [../]
  [./disp_x_101]
    type = SideAverageValue
    variable = disp_x
    boundary = 101
  [../]
  [./disp_x_102]
    type = SideAverageValue
    variable = disp_x
    boundary = 102
  [../]
  [./disp_x_103]
    type = SideAverageValue
    variable = disp_x
    boundary = 103
  [../]
  [./disp_x_104]
    type = SideAverageValue
    variable = disp_x
    boundary = 104
  [../]
  [./disp_x_105]
    type = SideAverageValue
    variable = disp_x
    boundary = 105
  [../]
  [./disp_x_106]
    type = SideAverageValue
    variable = disp_x
    boundary = 106
  [../]
  [./disp_y_101]
    type = SideAverageValue
    variable = disp_y
    boundary = 101
  [../]
  [./disp_y_102]
    type = SideAverageValue
    variable = disp_y
    boundary = 102
  [../]
  [./disp_y_103]
    type = SideAverageValue
    variable = disp_y
    boundary = 103
  [../]
  [./disp_y_104]
    type = SideAverageValue
    variable = disp_y
    boundary = 104
  [../]
  [./disp_y_105]
    type = SideAverageValue
    variable = disp_y
    boundary = 105
  [../]
  [./disp_y_106]
    type = SideAverageValue
    variable = disp_y
    boundary = 106
  [../]
  [./disp_z_101]
    type = SideAverageValue
    variable = disp_z
    boundary = 101
  [../]
  [./disp_z_102]
    type = SideAverageValue
    variable = disp_z
    boundary = 102
  [../]
  [./disp_z_103]
    type = SideAverageValue
    variable = disp_z
    boundary = 103
  [../]
  [./disp_z_104]
    type = SideAverageValue
    variable = disp_z
    boundary = 104
  [../]
  [./disp_z_105]
    type = SideAverageValue
    variable = disp_z
    boundary = 105
  [../]
  [./disp_z_106]
    type = SideAverageValue
    variable = disp_z
    boundary = 106
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
  automatic_scaling = true
  # end_time = 12960000
  # end_time = 51840000
  end_time = 38880000
  # 49766400
  l_max_its  = 50
  l_tol      = 1e-4
  nl_max_its = 10
  nl_rel_tol = 1e-8
  nl_abs_tol = 1e-10

  # [./TimeStepper]
  #   type = IterationAdaptiveDT
  #   dt = 36000
  #   # cutback_factor = 0.95
  #   # growth_factor = 1.05
  #   cutback_factor = 0.7
  #   # growth_factor = 1.2
  # [../]
[]

[Outputs]
  exodus         = true
  perf_graph     = true
  csv = true
  # file_base = asr_concrete_rebar_validation
  [./Console]
    type = Console
  [../]
[]

[Debug]
  show_var_residual_norms = true
[]
