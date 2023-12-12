# From PowerModelsACDC -> refer to it in the formulation
function constraint_voltage_dc(pm::_PM.AbstractWRMModel, n::Int)
    wdc = _PM.var(pm, n, :wdc)
    wdcr = _PM.var(pm, n, :wdcr)

    for (i,j) in _PM.ids(pm, n, :buspairsdc)
        JuMP.@constraint(pm.model, [ wdc[i]/sqrt(2), wdc[j]/sqrt(2), wdcr[(i,j)]/sqrt(2), wdcr[(i,j)]/sqrt(2)] in JuMP.RotatedSecondOrderCone() )
    end
end

function constraint_voltage_dc_ots(pm::_PM.AbstractWRMModel, n::Int)
    wdc = _PM.var(pm, n, :wdc_ne)
    wdc_frto = _PM.var(pm, n, :wdcr_ne)
    wdc_du_frto = _PM.var(pm, n, :wdcr_du)
    wdc_du_to = _PM.var(pm, n, :wdc_du_to)
    wdc_du_fr = _PM.var(pm, n, :wdc_du_fr)
    z  = _PM.var(pm, n, :z_ots_dc)

    for (l,i,j) in pm.ref[:it][:pm][:nw][n][:arcs_dcgrid_from_ne]
    wdc_to = []
    wdc_fr = []
    wdc_to, wdc_fr = _PMACDC.contraint_ohms_dc_branch_busvoltage_structure_W_ots(pm, n, i, j, wdc_du_to, wdc_du_fr)
    JuMP.@constraint(pm.model, [ wdc_du_to[l]/sqrt(2), wdc_du_fr[l]/sqrt(2), wdc_du_frto[l]/sqrt(2), wdc_du_frto[l]/sqrt(2)] in JuMP.RotatedSecondOrderCone() )
    end
end