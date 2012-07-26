function h = colortable_online(action, varargin)
  global COLORTABLE LUT DATA ROBOT;
  h.Initialize = @Initialize;
  h.update = @update;
  h.Color = @Color;
%
% main function for the colortable/lut training gui
%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% Sub-Function Definitions %%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


  function Initialize()
  % initialize the colortable gui
  % creates the gui and ui elements for the colortable trainer

    % create the gui
    hfig = gcf;
    clf;
    set(hfig, 'Name', 'UPenn Colortable Selection', ...
              'NumberTitle', 'off', ...
              'tag', 'Colortable', ...
              'MenuBar', 'none', ...
              'ToolBar', 'figure', ...
              'Color', [.8 .8 .8], ...
              'Colormap', gray(256), ...
							'KeyPressFcn',@KeyResponse);

    % set the figure window size
    scrsz = get(0, 'ScreenSize');
    figpos = get(hfig, 'Position');
    figpos(3:4) = [1100 600];
    figpos = [scrsz(3)/2 - figpos(3)/2, scrsz(4) - figpos(4)/2, figpos(3), figpos(4)];
    set(hfig, 'Position', figpos);

    % default image size 
%    DATA.size = [120 160];
    DATA.size = [240 320];

    % init mask 
    for icolor = 1:COLORTABLE.ncolor
      % clear out mask with new image
      DATA.mask_pos{icolor} = false(DATA.size);
      DATA.mask_neg{icolor} = false(DATA.size);
    end

    % default color threshold value
    DATA.cthreshold = 14;
    % array containing the training images
    DATA.montage = [];
    % index of the current color
    DATA.icolor = 1;

    % Toggle between masking mode and label preview mode
    DATA.viewmode = 0;

    DATA.LogFilePath = './logs';
    DATA.LogList = {};

    % standard options for all gui elements
    % do not allow callbacks to be interrupted
    Std.Interruptible = 'off';
    % if a callback is triggered while another is still being executed
    % queue that callback (run it after the current one is finished)
    Std.BusyAction = 'queue';

    % create the axis to display the current image
    DATA.ImagePanel = uipanel('Parent', hfig, ...
                              'Title', 'Images', ...
                              'BackgroundColor', [.8 .8 .8], ...
                              'Position', [.2 .15 .76 .8]);
    DATA.ImageAxes = subplot(2, 3, [1 2 4 5], ...
                           'Parent', DATA.ImagePanel, ...
                           'YDir', 'reverse', ...
                           'XLim', .5+[0 DATA.size(2)], ...
                           'YLim', .5+[0 DATA.size(1)], ...
                           'XTick', [], ...
                           'YTick', [], ...
                           'Position', [0.02 0.045 0.63 0.9], ...
                           'Units', 'Normalized');

    % create the handle for the image data
    DATA.Image = image(Std, ...
                        'Parent', DATA.ImageAxes, ...
                        'XData', [1 DATA.size(2)], ...
                        'YData', [1 DATA.size(1)], ...
                        'ButtonDownFcn', @Button, ...
                        'CData', []);

    DATA.MaskAxes = subplot(2, 3, 3, ...
                           'Parent', DATA.ImagePanel, ...
                           'YDir', 'reverse', ...
                           'XLim', .5+[0 DATA.size(2)], ...
                           'YLim', .5+[0 DATA.size(1)], ...
                           'XTick', [], ...
                           'YTick', [], ...
                           'Position', [0.67 0.520 0.313 0.425], ...
                           'Units', 'Normalized');

    % create the handle for the image data
    DATA.Mask = image(Std, ...
                        'Parent', DATA.MaskAxes, ...
                        'XData', [1 DATA.size(2)], ...
                        'YData', [1 DATA.size(1)], ...
                        'CData', []);

    DATA.LabelAxes = subplot(2, 3, 6, ...
                           'Parent', DATA.ImagePanel, ...
                           'YDir', 'reverse', ...
                           'XLim', .5+[0 DATA.size(2)], ...
                           'YLim', .5+[0 DATA.size(1)], ...
                           'XTick', [], ...
                           'YTick', [], ...
                           'Position', [0.67 0.045 0.313 0.425], ...
                           'Units', 'Normalized');

    % create the handle for the image data
    DATA.Label = image(Std, ...
                        'Parent', DATA.LabelAxes, ...
                        'XData', [1 DATA.size(2)], ...
                        'YData', [1 DATA.size(1)], ...
                        'CData', []);


    % create 'Load Montage' button
    %{
    DATA.LoadControl = uicontrol(Std, ...
                                  'Parent', hfig, ...
                                  'Style', 'pushbutton', ...
                                  'String', 'Load Montage (L)', ...
                                  'Callback','colortable_online(''LoadMontage'')', ...
                                  'Units', 'Normalized', ...
                                  'Position', [.025 .90 .15 .05]);
%}
    % create 'Save Colors' button
    DATA.SaveColor = uicontrol(Std, ...
                                  'Parent', hfig, ...
                                  'Style', 'pushbutton', ...
                                  'String', 'Save Colors (Q)', ...
                                  'Callback','colortable_online(''SaveColor'')', ...
                                  'Units', 'Normalized', ...
                                  'Position', [.025 .12 .15 .05]);

    % create 'Save lut' button
    DATA.SaveLut = uicontrol(Std, ...
                                  'Parent', hfig, ...
                                  'Style', 'pushbutton', ...
                                  'String', 'Save LUT (W)', ...
                                  'Callback','colortable_online(''SaveLUT'')', ...
                                  'Units', 'Normalized', ...
                                  'Position', [.025 .05 .15 .05]);

    % create 'Clear Selection' button
    DATA.ClearControl = uicontrol(Std, ...
                                  'Parent', hfig, ...
                                  'Style', 'pushbutton', ...
                                  'String', 'Clear Selection (C)', ...
                                  'Callback','colortable_online(''ClearSelection'')', ...
                                  'Units', 'Normalized', ...
                                  'Position', [.025 .35 .15 .05]);

    DATA.ClearLUT = uicontrol(Std, ...
                                  'Parent', hfig, ...
                                  'Style', 'pushbutton', ...
                                  'String', 'Clear Colortable', ...
                                  'Callback',@ClearLUT, ...
                                  'Units', 'Normalized', ...
                                  'Position', [.025 .50 .15 .05]);

%{
    % create the Forward arrow button
    DATA.ForwardControl = uicontrol(Std, ...
                                    'Parent', hfig, ...
                                    'Style', 'pushbutton', ...
                                    'String', '-> (D)', ...
                                    'Callback','colortable_online(''UpdateImage'',''forward'')', ...
                                    'Units', 'Normalized', ...
                                    'Position', [.65 .05 .07 .05]);

    % create the Backward arrow button
    DATA.BackwardControl = uicontrol(Std, ...
                                      'Parent', hfig, ...
                                      'Style', 'pushbutton', ...
                                      'String', '<- (S)', ...
                                      'Callback','colortable_online(''UpdateImage'',''backward'')', ...
                                      'Units', 'Normalized', ...
                                      'Position', [.4 .05 .07 .05]);

    % create the double Forward arrow button
    DATA.FastForwardControl = uicontrol(Std, ...
                                        'Parent', hfig, ...
                                        'Style', 'pushbutton', ...
                                        'String', '>> (F)', ...
                                        'Callback','colortable_online(''UpdateImage'',''fastforward'')', ...
                                        'Units', 'Normalized', ...
                                        'Position', [.75 .05 .07 .05]);

    % create the double Backward arrow button
    DATA.FastBackwardControl = uicontrol(Std, ...
                                          'Parent', hfig, ...
                                          'Style', 'pushbutton', ...
                                          'String', '<< (A)', ...
                                          'Callback','colortable_online(''UpdateImage'',''fastbackward'')', ...
                                          'Units', 'Normalized', ...
                                          'Position', [.3 .05 .07 .05]);

    % create the image index select edit box
    DATA.IndexControl = uicontrol(Std, ...
                                  'Parent', hfig, ...
                                  'Style', 'edit', ...
                                  'String', '1', ...
                                  'Callback','colortable_online(''UpdateImage'',str2num(get(gco,''String'')))', ...
                                  'Units', 'Normalized', ...
                                  'Position', [.5 .05 .1 .06]);
%}

    % create the selection array for the colors of interest
    for icolor = 1:COLORTABLE.ncolor,
      DATA.ColorControl(icolor) = uicontrol(Std, ...
                                             'Parent', hfig, ...
                                             'Style', 'radiobutton', ...
                                             'String', strcat(COLORTABLE.color_name{icolor},' (',num2str(icolor), ')'), ...
                                             'UserData', icolor, ...
                                             'Callback',@Color,...
                                             'Value', 0, ...
                                             'Units', 'Normalized', ...
                                             'Position', [.025 .88-.045*icolor .15 .05]);
    end

    % create the color selection threshold slider
    DATA.ThresholdControl = uicontrol(Std, ...
                                       'Parent', hfig, ...
                                       'Style', 'slider', ...
                                       'Min', 0, ...
                                       'Max', 128, ...
                                       'Value', DATA.cthreshold, ...
                                       'Callback','colortable_online(''UpdateThreshold'',get(gco,''Value''))', ...
                                       'Units', 'Normalized', ...
                                       'Position', [.025 .25 .15 .05]);

    % create the color selection threshold edit box
    DATA.ThresholdEdit = uicontrol(Std, ...
                                    'Parent', hfig, ...
                                    'Style', 'edit', ...
                                    'String', num2str(DATA.cthreshold), ...
                                    'Callback','colortable_online(''UpdateThreshold'',str2num(get(gco,''String'')))', ...
                                    'Units', 'Normalized', ...
                                    'Position', [.075 .20 .05 .05]);

    % create the color selection title label
    DATA.ThresholdLabel = uicontrol(Std, ...
                                    'Parent', hfig, ...
                                    'Style', 'text', ...
                                    'String','(E) Threshold (R)', ...
                                    'Units', 'Normalized', ...
                                    'Position', [.025 .30 .15 .035]);


    % set DATA as the gui userdata (so we can access the data later)
    set(hfig, 'UserData', DATA, 'Visible', 'on');
    drawnow;
    return;
  end
	
	function KeyResponse(h_obj, evt)
    if evt.Key == 'z' | evt.Key == 'Z'
      disp('Select Previous Log File');
      LoadMontage('Backward');
    elseif evt.Key == 'x' | evt.Key == 'X'
      disp('Select Next Log File');
      LoadMontage('Forward'); 
    elseif evt.Key == 'a' | evt.Key == 'A'
			disp('Fast Backwards');
			UpdateImage('fastbackward');
		elseif evt.Key == 's' | evt.Key == 'S'
			disp('Backward');
			UpdateImage('backward');
		elseif evt.Key == 'd' | evt.Key == 'D'
			disp('Forward');
			UpdateImage('forward');
		elseif evt.Key == 'f' | evt.Key == 'F'
			disp('Fast Forward');
			UpdateImage('fastforward');
		elseif evt.Key == 't' | evt.Key == 'T'
			disp('ToggleView?');
			ToggleView();
		elseif evt.Key == 'l' | evt.Key == 'L'
			disp('LoadMontage');
			LoadMontage();
		elseif evt.Key == 'c' | evt.Key == 'C'
			disp('ClearSelection');
			ClearSelection();
		elseif evt.Key == 'q' | evt.Key == 'Q'
			disp('SaveColor');
			SaveColor();
		elseif evt.Key == 'w' | evt.Key == 'W'
			disp('SaveLut');
			SaveLUT();
		elseif evt.Key == 'e' | evt.Key == 'E'
			disp('Lower Threshold');
			data = get(h_obj, 'UserData');
			value = get(data.ThresholdControl, 'Value');
			value = value - 1;
			UpdateThreshold(value);
		elseif evt.Key == 'r' | evt.Key == 'R'
			disp('Higher Threshold');
			data = get(h_obj, 'UserData');
			value = get(data.ThresholdControl, 'Value');
			value = value + 1;
			UpdateThreshold(value);
		elseif evt.Key >= '1' & evt.Key <= '7'
			Color(str2num(evt.Key));
		end
  end
				 

  function UpdateImage(index)
  % updates the image displayed in the gui

    % get the gui userdata
    hfig = gcbf;
    DATA = get(hfig, 'UserData');

    if isempty(DATA.montage)
      % if no montage has been loaded, do nothing
      return;
    end

    % get the number of images in the montage
    nMontage = size(DATA.montage, 4);

    % iImage is the index of the current image
    if ~isfield(DATA, 'iImage')
      % if it is not already set (i.e. the montage was just loaded)
      % then initialize it to one
      DATA.iImage = 1;
    end

    % did the index change? (go to next image)
    if (nargin >= 1)
      % update color score data
      colortable_merge(DATA);
      
      % determine new image index
      if strcmp(index,'forward') 
        % forward button was pressed
        DATA.iImage = DATA.iImage + 1;
      elseif strcmp(index,'fastforward')
        % fast forward button was pressed
        DATA.iImage = DATA.iImage + 10;
      elseif strcmp(index,'backward')
        % back button was pressed
        DATA.iImage = DATA.iImage - 1;
      elseif strcmp(index,'fastbackward')
        % fast back button was pressed
        DATA.iImage = DATA.iImage - 10;
      else
        % the index edit box was set
        DATA.iImage = index;
      end

      % max sure iImage is a valid index
      DATA.iImage = min(max(DATA.iImage, 1), nMontage);

      % the yuv color data for the current image
      DATA.yuv = DATA.montage(:, :, :, DATA.iImage);
      % convert the yuyv data into the index values
      % these are the indices of the LUT 
      DATA.cindex = yuv2index(DATA.yuv, COLORTABLE.size);

      % convert the yuv image to rgb so we can display it
      DATA.rgb = ycbcr2rgb(DATA.yuv);

      for icolor = 1:COLORTABLE.ncolor
        % clear out mask with new image
        DATA.mask_pos{icolor} = false(DATA.size);
        DATA.mask_neg{icolor} = false(DATA.size);
      end
    end

    % get the current colortable score
    score = colortable_score(DATA.cindex, DATA.icolor);

    % create the display image (visualizing the current mask)
    im_display = DATA.rgb;

    % visualize the current colortable score 
    if (DATA.icolor == 5) % white
      % special case for if the current color is white
      %   the generic way is not visible (white on white)
      im_display(:,:,2) = DATA.rgb(:,:,2) - uint8((score>0).*double(DATA.rgb(:,:,2)));
      im_display(:,:,3) = DATA.rgb(:,:,3) - uint8((score>0).*double(DATA.rgb(:,:,3)));
    else
      im_display(:,:,2) = DATA.rgb(:,:,2) + uint8(score.*double(255-DATA.rgb(:,:,2)));
    end

    % visualize current pos/neg selection masks
    im_display = rgbmask(im_display, DATA.mask_pos{DATA.icolor}, ...
                           DATA.mask_neg{DATA.icolor}, DATA.icolor);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Live label view
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if DATA.viewmode 
      class=LUT(DATA.cindex);
      cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
      cmap=[cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
      r_cast=cmap(:,1)*255;
      g_cast=cmap(:,2)*255;
      b_cast=cmap(:,3)*255;

      %class starts with 0, index starts with 1 
      im_display(:,:,1)=r_cast(class+1);
      im_display(:,:,2)=g_cast(class+1);
      im_display(:,:,3)=b_cast(class+1);
    end


    % set the display image data in the gui
    set(DATA.Image, 'CData', im_display);

    % update the current image index display
    set(DATA.IndexControl, 'String', num2str(DATA.iImage));

    set(hfig, 'UserData', DATA);
    return;
  end


  function Button(varargin)
  % callback for clicking on the image

    % get the gui userdata
    hfig = gcbf;

    % get the pointer position
    pt = get(gca,'CurrentPoint');
    ptx = round(pt(1,1));
    pty = round(pt(1,2));


    % select similar colored pixels based on the color selection threshold
    % mask is a binary array where selected pixels have a value of 1
    mask = rgbselect(DATA.rgb, ptx, pty, DATA.cthreshold);

    if strcmp(get(hfig,'SelectionType'),'normal')
      % left click
      % add selected pixels to the positive examples mask
      DATA.mask_pos{DATA.icolor} = DATA.mask_pos{DATA.icolor} | mask;
      % remove any selected pixels from the negative examples mask
      DATA.mask_neg{DATA.icolor} = DATA.mask_neg{DATA.icolor} & ~mask;
    elseif strcmp(get(hfig,'SelectionType'),'extend')
      % shift + left click
      % remove any selected pixels from the positive examples mask
      DATA.mask_pos{DATA.icolor} = DATA.mask_pos{DATA.icolor} & ~mask;
      % add selected pixels to the negative examples mask
      DATA.mask_neg{DATA.icolor} = DATA.mask_neg{DATA.icolor} | mask;
    elseif strcmp(get(hfig,'SelectionType'),'alt')
      % ctrl + left click
      % remove any selected pixels from the positive examples mask
      DATA.mask_pos{DATA.icolor} = DATA.mask_pos{DATA.icolor} & ~mask;
      % remove any selected pixels from the negative examples mask
      DATA.mask_neg{DATA.icolor} = DATA.mask_neg{DATA.icolor} & ~mask;
    end

    % update color score data
    colortable_merge(DATA);

    % convert yuv data into the index values of LUT
    DATA.cindex = yuv2index(DATA.yuv, COLORTABLE.size);

    % get the current colortable score
    score = colortable_score(DATA.cindex, DATA.icolor);

    colortable_smear;
    LUT = colortable_lut();

    lut_updated = ROBOT.matcmControl.get_lut_updated();
    disp('push shm from matlab');
    ROBOT.matcmControl.set_lut_updated(1 - lut_updated);
    ROBOT.vcmImage.set_lut(typecast(LUT,'double'));

  end

  function ClearLUT(varargin)
    nlut = size(COLORTABLE.score, 1);
    lut = zeros(262144, 1, 'uint8');
    lut_updated = ROBOT.matcmControl.get_lut_updated();
    disp('push emtpy shm from matlab');
    ROBOT.matcmControl.set_lut_updated(1 - lut_updated);
    ROBOT.vcmImage.set_lut(typecast(LUT,'double'));
  end




  function Color(varargin)
  % callback for selecting a color from the radio button array
    icolor = get(varargin{1}, 'UserData');
    % get the gui userdata
    hfig = gcbf;

    % get the selected color
    DATA.icolor = icolor;

    % update the selected color radio button (highlight)
    for icolor = 1:COLORTABLE.ncolor
      if (icolor == DATA.icolor)
        set(DATA.ColorControl(icolor), 'Value', 1);
      else
        set(DATA.ColorControl(icolor), 'Value', 0);
      end
    end
  end


  function UpdateThreshold(value)
  % callback for the color selection threshold slider

    % get the gui userdata
    hfig = gcbf;
    DATA = get(hfig, 'UserData');

    % set the new threshold value
    DATA.cthreshold = round(value);
    % update the threshold value display
    set(DATA.ThresholdControl, 'Value', DATA.cthreshold);
    set(DATA.ThresholdEdit, 'String', num2str(DATA.cthreshold));
    set(hfig, 'UserData', DATA);
    return;
  end


  function ClearSelection()
  % callback for the 'Clear Selection' button

    % get the gui userdata
    hfig = gcbf;
    DATA = get(hfig, 'UserData');

    % re-initialize masks
    DATA.mask_pos{DATA.icolor} = false(DATA.size);
    DATA.mask_neg{DATA.icolor} = false(DATA.size);

    set(hfig, 'UserData', DATA);

    UpdateImage();
    return;
  end


  function SaveColor()
  % callback for the 'Save Colors' button

    % open the save file gui
    [filename, pathname] = uiputfile('*.mat', 'Select colortable file to save');
    if (filename ~= 0)
      save([pathname filename],'COLORTABLE');
    end

    return;
  end

  function SaveLUT()
    % callback for the 'Save LUT' button

    % open file select gui
    [filename, pathname] = uiputfile('*.raw', 'Select lut file to save');

    if (filename ~= 0)
      colortable_smear;
      LUT = colortable_lut();
%      lut_montage(LUT);
      write_lut_file( LUT, [pathname filename] );
      disp(['Saved file ' filename])
    end

    return;
  end

  function update()
    cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
    cmap=[cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];

    % Show yuyv Image
    yuyv_type = ROBOT.vcmCamera.get_yuyvType();
    if yuyv_type == 1
      yuyv = ROBOT.get_yuyv(); 
    elseif yuyv_type == 2
      yuyv = ROBOT.get_yuyv2();
    elseif yuyv_type == 3
      yuyv = ROBOT.get_yuyv3();
    else
      return;
    end
    labelA = ROBOT.get_labelA(); 
    [ycbcr, rgb] = yuyv2rgb(yuyv);
    DATA.yuv = ycbcr;
    DATA.rgb = rgb;
    set(DATA.Image, 'CData', rgb);

    % Show Label A
    colormap(cmap);
    set(DATA.Label, 'CData', labelA);

    % Show mask
    mask_disp = DATA.mask_pos{DATA.icolor};
    set(DATA.Mask, 'CData', mask_disp);
    drawnow;
  end

end
