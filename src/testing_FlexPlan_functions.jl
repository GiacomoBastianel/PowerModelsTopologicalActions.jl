using PowerModels; const _PM = PowerModels
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
using FlexPlan; const _FP = FlexPlan
using Gurobi

gurobi = Gurobi.Optimizer
##### Testing FlexPlan.jl functions to build multinetwork model with scenarios
test_case_5_acdc = "case5_acdc.m"
data_file_5_acdc = joinpath(dirname(@__DIR__),"data_sources",test_case_5_acdc)

data_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)


_FP._initialize_dim()

_FP.add_dimension!(data_5_acdc, :hour, 24)
_FP.add_dimension!(data_5_acdc, :scenario, Dict(1 => Dict{String,Any}("probability"=>0.8),2 => Dict{String,Any}("probability"=>0.2)))

data_5_acdc["dim"][:prop][:hour]
data_5_acdc["dim"][:prop][:scenario]

result = _FP.simple_stoch_flex_tnep(data_5_acdc, _PM.DCPPowerModel, gurobi; setting=Dict("conv_losses_mp"=>false))