function constraint_ohms_dc_branch_busvoltage_structure_W_ots(pm::_PM.AbstractPowerModel, n::Int, f_bus, t_bus, wdc_to, wdc_fr)
    for i in _PM.ids(pm, n, :busdc)
        if t_bus == i
            wdc_to = _PM.var(pm, n, :wdc, t_bus)
        end
        if f_bus == i
            wdc_fr = _PM.var(pm, n, :wdc, f_bus)
        end
    end
    return wdc_to, wdc_fr
end