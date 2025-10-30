# usc's beam (refer NEUP report Project No. 15-8458)
# Cement: Type II/IV low alkali Portland cement (alkali content < 0.6%
# Sand: Reactive from El Paso (mortar bar test shows 0.6% and 0.8% ASR expansion at 16 days and 25 days, respectively)
# Coarse aggregate: 3/4" and 3/8" aggregate (crushed rock) are used
# Concrete mix proportions:
# Batch  Beam  Cement    water     CA(3/4")  CA(3/8")  FA    Alkali
# 1      4.    1.0       0.52      3.6       0.7       2.5   0.0
# 2      5.    1.0       0.52      3.6       0.7       2.5   0.0
# 3      6.    1.0       0.52      3.6       0.7       2.5   0.0125
# Environmental conditions:
#   1. Lab environment
#   2. Covered outdoor storage with periodic (twice per week) water spray
# curing duration = 14 days
# curing environment = wrapped in plastic sheet
#
# Compressive strength (MPa) (approx values)
# batch1_lab:     38
# batch2_outside: 37
# batch3_outside: 34

[GlobalParams]
 displacements = 'disp_x disp_y disp_z'
 volumetric_locking_correction = true
[]

[Mesh]
  file = usc_beam4_asr_paper.e
  construct_side_list_from_node_list = true
  block_id = '2 3 4 5 6 7 8'
  block_name = 'long_no8 long_no5 trans_no5 trans_no3 left_span mid_span right_span'
  boundary_id = '1 2 3 4 5 6 7 8 9 10'
  boundary_name = 'back_left_bot front_left_bot back_right_bot all_surfaces left_span_z_surfaces left_span_y_surfaces mid_span_z_surfaces mid_span_y_surfaces right_span_z_surfaces right_span_y_surfaces'
[]

[Modules/TensorMechanics/Master]
  [concrete]
    block = 'left_span mid_span right_span'
    strain = FINITE
    add_variables = true
    eigenstrain_names = 'asr_expansion thermal_expansion'
    generate_output = 'stress_xx stress_yy stress_zz stress_xy stress_yz stress_zx vonmises_stress hydrostatic_stress elastic_strain_xx elastic_strain_yy elastic_strain_zz strain_xx strain_yy strain_zz'
    save_in = 'resid_x resid_y resid_z'
  []
[]

[Modules/TensorMechanics/LineElementMaster]
  [longitudinal_rebar_no8_block]
    block = 'long_no8'
    truss = true
    area = area_long_no8
    displacements = 'disp_x disp_y disp_z'
    save_in = 'resid_x resid_y resid_z'
  []
  [longitudinal_rebar_no5_block]
    block = 'long_no5'
    truss = true
    area = area_long_no5
    displacements = 'disp_x disp_y disp_z'
    save_in = 'resid_x resid_y resid_z'
  []
  [transverse_rebar_no5_block]
    block = 'trans_no5'
    truss = true
    area = area_trans_no5
    displacements = 'disp_x disp_y disp_z'
    save_in = 'resid_x resid_y resid_z'
  []
  [transverse_rebar_no3_block]
    block = 'trans_no3'
    truss = true
    area = area_trans_no3
    displacements = 'disp_x disp_y disp_z'
    save_in = 'resid_x resid_y resid_z'
  []
[]

[Constraints/EqualValueEmbeddedConstraint/Batch]
  secondary = 'long_no8 long_no5 trans_no5 trans_no3'
  primary = 'left_span mid_span right_span'
  variable = 'disp_x disp_y disp_z'
  penalty = 1e12
  formulation = penalty
[]

[Variables]
  [T]
    order = FIRST
    family = LAGRANGE
    initial_condition = 23.0
  []
  [rh]
    order = FIRST
    family = LAGRANGE
    initial_condition = 0.6
    block = 'left_span mid_span right_span'
  []
[]

[Kernels]
  [T_td]
    type = ConcreteThermalTimeIntegration
    variable = T
    block = 'left_span mid_span right_span'
  []
  [T_diff]
    type = ConcreteThermalConduction
    variable = T
    block = 'left_span mid_span right_span'
  []
  [T_conv]
    type = ConcreteThermalConvection
    variable = T
    relative_humidity = rh
    block = 'left_span mid_span right_span'
  []
  [T_adsorption]
    type = ConcreteLatentHeat
    variable = T
    H = rh
    block = 'left_span mid_span right_span'
  []
  [rh_td]
    type = ConcreteMoistureTimeIntegration
    variable = rh
    block = 'left_span mid_span right_span'
  []
  [rh_diff]
    type = ConcreteMoistureDiffusion
    variable = rh
    temperature = T
    block = 'left_span mid_span right_span'
  []
  [heat_dT_long_no8]
    type = TrussHeatConductionTimeDerivative
    variable = T
    block = 'long_no8'
    area = area_long_no8
  []
  [heat_conduction_long_no8]
    type = TrussHeatConduction
    variable = T
    block = 'long_no8'
    area = area_long_no8
  []
  [heat_dT_long_no5]
    type = TrussHeatConductionTimeDerivative
    variable = T
    block = 'long_no5'
    area = area_long_no5
  []
  [heat_conduction_long_no5]
    type = TrussHeatConduction
    variable = T
    block = 'long_no5'
    area = area_long_no5
  []
  [heat_dT_trans_no5]
    type = TrussHeatConductionTimeDerivative
    variable = T
    block = 'trans_no5'
    area = area_trans_no5
  []
  [heat_conduction_trans_no5]
    type = TrussHeatConduction
    variable = T
    block = 'trans_no5'
    area = area_trans_no5
  []
  [heat_dT_trans_no3]
    type = TrussHeatConductionTimeDerivative
    variable = T
    block = 'trans_no3'
    area = area_trans_no3
  []
  [heat_conduction_trans_no3]
    type = TrussHeatConduction
    variable = T
    block = 'trans_no3'
    area = area_trans_no3
  []
[]

[AuxVariables]
  [resid_x]
  []
  [resid_y]
  []
  [resid_z]
  []
  [ASR_ex]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_vstrain]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_strain_xx]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_strain_yy]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_strain_zz]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_strain_xy]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_strain_yz]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_strain_zx]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_Gamma_t]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_Gamma_c]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [ASR_gH]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [volumetric_strain]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [thermal_strain_xx]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [thermal_strain_yy]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [thermal_strain_zz]
    order = CONSTANT
    family = MONOMIAL
    block = 'left_span mid_span right_span'
  []
  [thermal_conductivity]
    order = CONSTANT
    family = Monomial
    block = 'left_span mid_span right_span'
  []
  [thermal_capacity]
    order = CONSTANT
    family = Monomial
    block = 'left_span mid_span right_span'
  []
  [humidity_diffusivity]
    order = CONSTANT
    family = Monomial
    block = 'left_span mid_span right_span'
  []
  [damage_index]
    order = CONSTANT
    family = Monomial
    block = 'left_span mid_span right_span'
  []

  [area_trans_no3]
    order = CONSTANT
    family = MONOMIAL
  []
  [area_long_no5]
    order = CONSTANT
    family = MONOMIAL
  []
  [area_trans_no5]
    order = CONSTANT
    family = MONOMIAL
  []
  [area_long_no8]
    order = CONSTANT
    family = MONOMIAL
  []
[]

[AuxKernels]
  [ASR_ex]
    type = MaterialRealAux
    variable = ASR_ex
    block = 'left_span mid_span right_span'
    property = ASR_extent
    execute_on = 'timestep_end'
  []
  [ASR_vstrain]
    type = MaterialRealAux
    block = 'left_span mid_span right_span'
    variable = ASR_vstrain
    property = ASR_volumetric_strain
    execute_on = 'timestep_end'
  []
  [ASR_strain_xx]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = asr_expansion
    variable = ASR_strain_xx
    index_i = 0
    index_j = 0
    execute_on = 'timestep_end'
  []
  [ASR_strain_yy]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = asr_expansion
    variable = ASR_strain_yy
    index_i = 1
    index_j = 1
    execute_on = 'timestep_end'
  []
  [ASR_strain_zz]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = asr_expansion
    variable = ASR_strain_zz
    index_i = 2
    index_j = 2
    execute_on = 'timestep_end'
  []
  [ASR_strain_xy]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = asr_expansion
    variable = ASR_strain_xy
    index_i = 0
    index_j = 1
    execute_on = 'timestep_end'
  []
  [ASR_strain_yz]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = asr_expansion
    variable = ASR_strain_yz
    index_i = 1
    index_j = 2
    execute_on = 'timestep_end'
  []
  [ASR_strain_zx]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = asr_expansion
    variable = ASR_strain_zx
    index_i = 0
    index_j = 2
    execute_on = 'timestep_end'
  []
  [ASR_Gamma_t]
    type = MaterialRealAux
    block = 'left_span mid_span right_span'
    variable = ASR_Gamma_t
    property = Gamma_t
    execute_on = 'timestep_end'
  []
  [ASR_Gamma_c]
    type = MaterialRealAux
    block = 'left_span mid_span right_span'
    variable = ASR_Gamma_c
    property = Gamma_c
    execute_on = 'timestep_end'
  []
  [ASR_gH]
    type = MaterialRealAux
    block = 'left_span mid_span right_span'
    variable = ASR_gH
    property = gH
    execute_on = 'timestep_end'
  []
  [thermal_strain_xx]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = thermal_expansion
    variable = thermal_strain_xx
    index_i = 0
    index_j = 0
    execute_on = 'timestep_end'
  []
  [thermal_strain_yy]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = thermal_expansion
    variable = thermal_strain_yy
    index_i = 1
    index_j = 1
    execute_on = 'timestep_end'
  []
  [thermal_strain_zz]
    type = RankTwoAux
    block = 'left_span mid_span right_span'
    rank_two_tensor = thermal_expansion
    variable = thermal_strain_zz
    index_i = 2
    index_j = 2
    execute_on = 'timestep_end'
  []
  [volumetric_strain]
    type = RankTwoScalarAux
    scalar_type = VolumetricStrain
    rank_two_tensor = total_strain
    variable = volumetric_strain
    block = 'left_span mid_span right_span'
  []
  [k]
    type = MaterialRealAux
    variable = thermal_conductivity
    property = thermal_conductivity
    execute_on = 'timestep_end'
    block = 'left_span mid_span right_span'
  []
  [capacity]
    type = MaterialRealAux
    variable = thermal_capacity
    property = thermal_capacity
    execute_on = 'timestep_end'
    block = 'left_span mid_span right_span'
  []
  [rh_duff]
    type = MaterialRealAux
    variable = humidity_diffusivity
    property = humidity_diffusivity
    execute_on = 'timestep_end'
    block = 'left_span mid_span right_span'
  []
  [damage_index]
    type = MaterialRealAux
    block = 'left_span mid_span right_span'
    variable = damage_index
    property = damage_index
    execute_on = timestep_end
  []
  [area_trans_no3]
    type = ConstantAux
    block = 'trans_no3'
    variable = area_trans_no3
    value = 71e-6
    execute_on = 'initial timestep_begin'
  []
  [area_long_no5]
    type = ConstantAux
    block = 'long_no5'
    variable = area_long_no5
    value = 200e-6
    execute_on = 'initial timestep_begin'
  []
  [area_trans_no5]
    type = ConstantAux
    block = 'trans_no5'
    variable = area_trans_no5
    value = 200e-6
    execute_on = 'initial timestep_begin'
  []
  [area_long_no8]
    type = ConstantAux
    block = 'long_no8'
    variable = area_long_no8
    value = 509e-6
    execute_on = 'initial timestep_begin'
  []
[]

[Functions]
 [ramp_temp]
   type = PiecewiseLinear
   x = '5443200	5529600	5788800	5875200	5961600	6480000	6652800	7689600	7948800	8294400	8553600	8899200	9590400	9763200	10108800	10627200	10713600	10972800	11318400	11577600	11750400	12182400	12614400	13132800	13392000	15811200	16156800	23068800	23414400	23673600	24019200	24278400	24624000	24883200	25228800	25488000	25833600	26092800	26438400	26697600	27043200	27302400	27648000	27820800	28080000	28339200	28684800	28944000	29289600	29548800	29894400	30153600	30499200	31104000	31363200	31708800	31968000	32400000	32572800	32918400	33177600	33782400	34128000	34387200	34819200	34992000	35337600	35596800	35942400	36201600	36460800	37152000	41990400'
   y = '22.76666667	22.93333333	23.03333333	23.2	23.23333333	22.30666667	22.86666667	22.53333333	23.05333333	22.8	23.65333333	23.86666667	23.89333333	24.06666667	21.49333333	20.74666667	22.28	23.37333333	23.01333333	23.09333333	22.94666667	23.53333333	22	24.14	24.1	23.52	22.86666667	23.84	24	23.54666667	24.08	23.6	23.2	22.90666667	22.61333333	23.05333333	23.85333333	24.02666667	24.18666667	24.17333333	24.56	24.06666667	24.04	23.98666667	24.50666667	24.33333333	24.64	23.62666667	23.81333333	23.30666667	23.21333333	23.70666667	24.06666667	24.04	23.08	23.08	21.49333333	23.78666667	24.69333333	23.98666667	24.06666667	23.65333333	23.28	22.4	23.28	23.37333333	23.38666667	24.09333333	24.13333333	23.68	24.26666667	22.8	23.36'
 []
 [ramp_humidity]
   type = PiecewiseLinear
   x = '5443200	5529600	5788800	5875200	5961600	6480000	6652800	7689600	7948800	8294400	8553600	8899200	9590400	9763200	10108800	10627200	10713600	10972800	11318400	11577600	11750400	12182400	12614400	13132800	13392000	15811200	16156800	16416000	16761600	17020800	17366400	17625600	17971200	18230400	18576000	18835200	19180800	19440000	19785600	20044800	20390400	20649600	20995200	21254400	21600000	21859200	22204800	22464000	22809600	23068800	23414400	23673600	24019200	24278400	24624000	24883200	25228800	25488000	25833600	26092800	26438400	26697600	27043200	27302400	27648000	27820800	28080000	28339200	28684800	28944000	29289600	29548800	29894400	30153600	30499200	31104000	31363200	31708800	31968000	32400000	32572800	32918400	33177600	33782400	34128000	34387200	34819200	34992000	35337600	35596800	35942400	36201600	36460800	36806400	37152000	37411200	37756800	38016000	38361600	38620800	39916800	40176000	40521600	40780800	41126400	41385600	41731200	41990400	42336000	42595200	42940800	43200000	44150400	44755200	45964800	47174400	49593600'
   y = '0.645	0.6405	0.7035	0.724	0.655	0.71715	0.70595	0.64655	0.7529	0.5937	0.71865	0.7749	0.69165	0.7503	0.63625	0.7356	0.73365	0.72215	0.7232	0.5786	0.60365	0.65865	0.7056	0.6654	0.677	0.70405	0.7471	0.5504	0.5935	0.6041	0.5501	0.6049	0.5737	0.5737	0.5565	0.579	0.6359	0.6498	0.592	0.6316	0.5268	0.5428	0.5719	0.5698	0.558	0.5563	0.5676	0.5589	0.5245	0.521	0.67125	0.67835	0.67615	0.6476	0.6357	0.64165	0.6466	0.67785	0.6761	0.63415	0.6711	0.67215	0.66075	0.6338	0.55545	0.5838	0.6676	0.66155	0.6647	0.63045	0.6644	0.5896	0.53645	0.57885	0.61215	0.577	0.6295	0.54275	0.6234	0.6214	0.54425	0.64425	0.61305	0.498	0.4804	0.48765	0.6013	0.6152	0.61525	0.6392	0.4079	0.6904	0.4105	0.4454	0.4329	0.3574	0.3171	0.4285	0.2976	0.3001	0.4255	0.3549	0.2795	0.3903	0.4187	0.4583	0.4216	0.3801	0.4164	0.476	0.3829	0.4346	0.5552	0.5171	0.4936	0.527	0.551'
 []
[]

[Materials]
 [concrete]
   type = ConcreteThermalMoisture
   block = 'left_span mid_span right_span'
   # setup thermal transport models and parameters
   # options available: CONSTANT ASCE-1992 KODUR-2004 EUROCODE-2004 KIM-2003
   thermal_model = KODUR-2004
   aggregate_type = Siliceous               #options: Siliceous Carbonate

   # setup moisture transport models and parameters
   # options available: Bazant Mensi Xi
   moisture_model = Xi
   cement_type = 2 # options: 1 2 3 4
   aggregate_vol_fraction = 0.7 # used in Xi's moisture diffusivity model
   concrete_cure_time = 14.0 # curing time in (days)
   aggregate_pore_type = dense # options: dense porous

   # concrete mix parameters
   cement_mass = 354.0 #mass of cement (kg) per m^3 of concrete
   aggregate_mass = 1877.0 #mass of aggregate (kg) per m^3 of concrete
   water_to_cement_ratio = 0.53
   ref_density = 2231.0 # in kg/m^3
   ref_specific_heat = 1100.0 # in J/(Kg.0C)
   ref_thermal_conductivity = 3 # in W/(m.0C)

   # coupled nonlinear variables
   relative_humidity = rh
   temperature = T
 []
 [creep]
   type = LinearViscoelasticStressUpdate
   block = 'left_span mid_span right_span'
 []
 [logcreep]
   type = ConcreteLogarithmicCreepModel
   block = 'left_span mid_span right_span'
   poissons_ratio = 0.22
   youngs_modulus = 37.3e9
   recoverable_youngs_modulus = 37.3e9
   recoverable_viscosity = 1
   long_term_viscosity = 1
   long_term_characteristic_time = 1
   humidity = rh
   temperature = T
   activation_temperature = 23.0
 []
 [ASR_expansion]
   type = ConcreteASREigenstrain
   block = 'left_span mid_span right_span'
   expansion_type = Anisotropic

   reference_temperature = 23.0      # parameter to play
   temperature_unit = Celsius
   max_volumetric_expansion = 1.125e-2  # parameter to play

   characteristic_time = 100       # parameter to play
   latency_time = 50        # parameter to play
   characteristic_activation_energy = 5400.0
   latency_activation_energy = 9400.0
   stress_latency_factor = 1.0

   compressive_strength = 38.0e6
   compressive_stress_exponent = 0.0
   expansion_stress_limit = 8.0e6

   tensile_strength = 3.8e6
   tensile_retention_factor = 1.0
   tensile_absorption_factor = 1.0

   ASR_dependent_tensile_strength = false
   residual_tensile_strength_fraction = 1.0

   temperature = T
   relative_humidity = rh
   rh_exponent = 1.0
   eigenstrain_name = asr_expansion
   absolute_tolerance = 1e-10
   output_iteration_info_on_error = true
 []
 [thermal_strain_concrete]
   type = ComputeThermalExpansionEigenstrain
   block = 'left_span mid_span right_span'
   temperature = T
   thermal_expansion_coeff = 8.0e-6
   stress_free_temperature = 10.6
   eigenstrain_name = thermal_expansion
 []
 [ASR_damage_concrete]
   type = ConcreteASRMicrocrackingDamage
   residual_youngs_modulus_fraction = 0.1
   block = 'left_span mid_span right_span'
 []
 [stress]
   type = ComputeMultipleInelasticStress
   block = 'left_span mid_span right_span'
   inelastic_models = 'creep'
   damage_model = ASR_damage_concrete
 []
 [truss]
   type = LinearElasticTruss
   block = 'long_no8 long_no5 trans_no5 trans_no3'
   youngs_modulus = 2e11
   temperature = T
   thermal_expansion_coeff = 11.3e-6
   temperature_ref = 10.6
 []
 [truss_thermal_prop]
   type = GenericConstantMaterial
   block = 'long_no8 long_no5 trans_no5 trans_no3'
   prop_names =  'thermal_conductivity specific_heat density'
   prop_values = '45                 446           7850' # W/(m K), J/(kg K), kg/m^3
 []
[]

[UserObjects]
 [visco_update]
   type = LinearViscoelasticityManager
   block = 'left_span mid_span right_span'
   viscoelastic_model = logcreep
 []
[]

[BCs]
 [x_disp]
   type = DirichletBC
   variable = disp_x
   boundary = 'back_left_bot'
   value = 0.0
 []
 [y_disp]
   type = DirichletBC
   variable = disp_y
   boundary = 'back_left_bot front_left_bot'
   value = 0.0
 []
 [z_disp]
   type = DirichletBC
   variable = disp_z
   boundary = 'back_left_bot front_left_bot back_right_bot'
   value = 0.0
 []
 [T]
   type = FunctionDirichletBC
   variable = T
   boundary = 'all_surfaces'
   function = ramp_temp
 []
 [rh]
   type = FunctionDirichletBC
   variable = rh
   boundary = 'all_surfaces'
   function = ramp_humidity
 []
[]

[Postprocessors]
  [ASR_vstrain_left_span]
    type = ElementAverageValue
    variable = ASR_vstrain
    block = 'left_span'
  []
  [ASR_strain_xx_left_span]
    type = ElementAverageValue
    variable = ASR_strain_xx
    block = 'left_span'
  []
  [ASR_strain_yy_left_span]
    type = ElementAverageValue
    variable = ASR_strain_yy
    block = 'left_span'
  []
  [ASR_strain_zz_left_span]
    type = ElementAverageValue
    variable = ASR_strain_zz
    block = 'left_span'
  []
  [ASR_ext_left_span]
    type = ElementAverageValue
    variable = ASR_ex
    block = 'left_span'
  []
  [Gamma_t_left_span]
    type = ElementAverageValue
    variable = ASR_Gamma_t
    block = 'left_span'
  []
  [Gamma_c_left_span]
    type = ElementAverageValue
    variable = ASR_Gamma_c
    block = 'left_span'
  []
  [gH_left_span]
    type = ElementAverageValue
    variable = ASR_gH
    block = 'left_span'
  []
  [rh_left_span]
    type = ElementAverageValue
    variable = rh
    block = 'left_span'
  []
  [T_left_span]
    type = ElementAverageValue
    variable = T
    block = 'left_span'
  []
  [vonmises_left_span]
    type = ElementAverageValue
    variable = vonmises_stress
    block = 'left_span'
  []

  [ASR_vstrain_mid_span]
    type = ElementAverageValue
    variable = ASR_vstrain
    block = 'mid_span'
  []
  [ASR_strain_xx_mid_span]
    type = ElementAverageValue
    variable = ASR_strain_xx
    block = 'mid_span'
  []
  [ASR_strain_yy_mid_span]
    type = ElementAverageValue
    variable = ASR_strain_yy
    block = 'mid_span'
  []
  [ASR_strain_zz_mid_span]
    type = ElementAverageValue
    variable = ASR_strain_zz
    block = 'mid_span'
  []
  [ASR_ext_mid_span]
    type = ElementAverageValue
    variable = ASR_ex
    block = 'mid_span'
  []
  [Gamma_t_mid_span]
    type = ElementAverageValue
    variable = ASR_Gamma_t
    block = 'mid_span'
  []
  [Gamma_c_mid_span]
    type = ElementAverageValue
    variable = ASR_Gamma_c
    block = 'mid_span'
  []
  [gH_mid_span]
    type = ElementAverageValue
    variable = ASR_gH
    block = 'mid_span'
  []
  [rh_mid_span]
    type = ElementAverageValue
    variable = rh
    block = 'mid_span'
  []
  [T_mid_span]
    type = ElementAverageValue
    variable = T
    block = 'mid_span'
  []
  [vonmises_mid_span]
    type = ElementAverageValue
    variable = vonmises_stress
    block = 'mid_span'
  []

  [ASR_vstrain_right_span]
    type = ElementAverageValue
    variable = ASR_vstrain
    block = 'right_span'
  []
  [ASR_strain_xx_right_span]
    type = ElementAverageValue
    variable = ASR_strain_xx
    block = 'right_span'
  []
  [ASR_strain_yy_right_span]
    type = ElementAverageValue
    variable = ASR_strain_yy
    block = 'right_span'
  []
  [ASR_strain_zz_right_span]
    type = ElementAverageValue
    variable = ASR_strain_zz
    block = 'right_span'
  []
  [ASR_ext_right_span]
    type = ElementAverageValue
    variable = ASR_ex
    block = 'right_span'
  []
  [Gamma_t_right_span]
    type = ElementAverageValue
    variable = ASR_Gamma_t
    block = 'right_span'
  []
  [Gamma_c_right_span]
    type = ElementAverageValue
    variable = ASR_Gamma_c
    block = 'right_span'
  []
  [gH_right_span]
    type = ElementAverageValue
    variable = ASR_gH
    block = 'right_span'
  []
  [rh_right_span]
    type = ElementAverageValue
    variable = rh
    block = 'right_span'
  []
  [T_right_span]
    type = ElementAverageValue
    variable = T
    block = 'right_span'
  []
  [vonmises_right_span]
    type = ElementAverageValue
    variable = vonmises_stress
    block = 'right_span'
  []

  [CL_L]
    type = AverageExtensionRatio
    displacements = 'disp_x disp_y disp_z'
    first_point = '+0.075 -0.075 +0.1525
                   +0.075 -0.075 -0.1525'
    last_point = '-0.075 -0.075 +0.1525
                  -0.075 -0.075 -0.1525'
  []
  [CL_U]
    type = AverageExtensionRatio
    displacements = 'disp_x disp_y disp_z'
    first_point = '+0.075 +0.075 +0.1525
                   +0.075 +0.075 -0.1525'
    last_point = '-0.075 +0.075 +0.1525
                  -0.075 +0.075 -0.1525'
  []
  [CT_C]
    type = AverageExtensionRatio
    displacements = 'disp_x disp_y disp_z'
    first_point = '-0.075 +0.075 +0.1525
                   -0.075 +0.075 -0.1525'
    last_point = '-0.075 -0.075 +0.1525
                  -0.075 -0.075 -0.1525'
  []
  [CT_E]
    type = AverageExtensionRatio
    displacements = 'disp_x disp_y disp_z'
    first_point = '+0.075 +0.075 +0.1525
                   +0.075 +0.075 -0.1525'
    last_point = '+0.075 -0.075 +0.1525
                  +0.075 -0.075 -0.1525'
  []
  [temp]
    type = SideAverageValue
    variable = T
    boundary = 'all_surfaces'
  []
  [humidity]
    type = SideAverageValue
    variable = rh
    boundary = 'all_surfaces'
  []
  [T_top]
    type = PointValue
    variable = T
    point = '-2.2005 0.195 -0.1145'
  []
  [T_mid]
    type = PointValue
    variable = T
    point = '-2.2005 0.07 -0.1145'
  []
  [T_bot]
    type = PointValue
    variable = T
    point = '-2.2005 -0.055 -0.1145'
  []
  [rh_top]
    type = PointValue
    variable = rh
    point = '-2.2005 0.195 -0.1145'
  []
  [rh_mid]
    type = PointValue
    variable = rh
    point = '-2.2005 0.07 -0.1145'
  []
  [rh_bot]
    type = PointValue
    variable = rh
    point = '-2.2005 -0.055 -0.1145'
  []
  [surfaceAvg_left_x]
    type = SideAverageValue
    variable = disp_x
    boundary = 'left_span_z_surfaces left_span_y_surfaces'
  []
  [surfaceAvg_left_y]
    type = SideAverageValue
    variable = disp_y
    boundary = 'left_span_z_surfaces'
  []
  [surfaceAvg_mid_x]
    type = SideAverageValue
    variable = disp_x
    boundary = 'mid_span_z_surfaces mid_span_y_surfaces'
  []
  [surfaceAvg_mid_y]
    type = SideAverageValue
    variable = disp_y
    boundary = 'mid_span_z_surfaces'
  []
  [surfaceAvg_right_x]
    type = SideAverageValue
    variable = disp_x
    boundary = 'right_span_z_surfaces right_span_y_surfaces'
  []
  [surfaceAvg_right_y]
    type = SideAverageValue
    variable = disp_y
    boundary = 'right_span_z_surfaces'
  []
[]

[Executioner]
 type = Transient
 start_time = 1209600 # 28 days
 dt = 86400 # 1 day
 end_time = 51840000 # 600 days

 # solve_type = 'NEWTON'
 # line_search = none
 # petsc_options_iname = '-pc_type'
 # petsc_options_value = 'lu'
 # petsc_options = '-snes_converged_reason'
 # nl_max_its = 100
 # nl_abs_tol = 1.E-5
 # nl_rel_tol = 1E-3

 solve_type = 'PJFNK'
 line_search = none
 petsc_options = '-ksp_snes_ew -snes_view'
 petsc_options_iname = '-pc_type -pc_hypre_type -ksp_gmres_restart -snes_ls -pc_hypre_boomeramg_strong_threshold'
 petsc_options_value = 'hypre boomeramg 201 cubic 0.7'
 automatic_scaling = true
 l_max_its = 10
 nl_max_its = 20
 l_tol = 1e-3
 l_abs_tol = 1e-5
 nl_rel_tol = 1e-5
 nl_abs_tol = 5e-5
[]

[Outputs]
 # perf_graph = true
 csv = true
 exodus = true
[]
