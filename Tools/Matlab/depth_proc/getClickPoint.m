function [ clickxy ] = getClickPoint( ui, im )
clickxy = [];

if ui.clickType > 0
    if isempty(ui.clickxy)
        fiugre(1), 
        imagesc(im); axis equal; axis([1 size(im,2) 1 size(im,1)]); 
        clickxy = ginput();
        clickxy = clickxy(end,:); % use the last point only
    else
        clickxy = ui.clixkxy;
    end
end

end

