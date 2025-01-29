# DCP formulation
function constraint_converter_losses(pm::_PM.AbstractDCPModel, n::Int,  i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    v = 1 #pu, assumption to approximate current
    cm_conv_ac = pconv_ac/v # can actually be negative, not a very nice model...
    #if pm.setting["conv_losses_mp"] == true
    #    JuMP.@constraint(pm.model, pconv_ac + pconv_dc == a + b*cm_conv_ac)
    #else
        JuMP.@constraint(pm.model, pconv_ac + pconv_dc >=   a + b*cm_conv_ac)
        JuMP.@constraint(pm.model, pconv_ac + pconv_dc >=  (a - b*cm_conv_ac))
        JuMP.@constraint(pm.model, pconv_ac + pconv_dc <= plmax)
    #end
end