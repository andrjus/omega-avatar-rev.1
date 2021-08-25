classdef avatar_coord < handle
	properties
        owner        matlab.ui.Figure
        board     matlab.ui.container.Panel
        lblCaption       matlab.ui.control.Label
        edDeseired      matlab.ui.control.NumericEditField
        sliderDeseired  matlab.ui.control.Slider
        lblActual       matlab.ui.control.Label
        gaugeActual     matlab.ui.control.LinearGauge
        text_h =  22;
        marker_w =  40;
        number_w =  60;
        margin_w = 5;
        margin_h = 5;
        slider_w =  400;
        board_h = 60;
        board_w = 550;
        range = [];
        required = 0.0;
        actual = 0.0;
        index = 0;
	end
	methods
        function  setRequired(handle, value)
            if value >  handle.range.required.max
                handle.required = handle.range.required.max;
            else
                if value < handle.range.required.min
                    handle.required  = handle.range.required.min;
                else 
                   handle.required = value;
                end
            end
            handle.edDeseired.Value = handle.required;
            handle.sliderDeseired.Value = handle.required;
        end
        
        function  setActual(handle, value)
            handle.actual = value;            
            handle.lblActual.Text = num2str(value,"%2.2f");
            if value >  handle.range.actual.max
                handle.gaugeActual.Value = handle.range.actual.max;
            else
                if value < handle.range.actual.min
                    handle.gaugeActual.Value = handle.range.actual.min;
                else 
                    handle.gaugeActual.Value = value;
                end
            end
        end
       
       function handle = avatar_coord( gui, x0, y0 ,index ,  coord)
            handle.board_w =  handle.marker_w + handle.number_w + handle.slider_w + 4 * handle.margin_w;
            handle.owner = gui.UIFigure;
            handle.index = index;
            handle.range = coord.range;
            
            % Create Panel
            handle.board = uipanel(handle.owner);
            handle.board.Title = '';
            handle.board.Position = [x0 y0 handle.board_w handle.board_h];

            x= handle.margin_w;
            y= handle.board_h - handle.margin_h - handle.text_h;
            
            handle.lblCaption = uilabel(handle.board);
            handle.lblCaption.HorizontalAlignment = 'right';
            handle.lblCaption.Position = [x y handle.marker_w handle.text_h];
            handle.lblCaption.Text = coord.caption;
            x = x + handle.marker_w + handle.margin_w;


            % Create edDeseired
            handle.edDeseired = uieditfield(handle.board, 'numeric');
            handle.edDeseired.ValueChangedFcn = @handle.sliderDeseiredChanging;
            handle.edDeseired.Position = [x y handle.number_w handle.text_h];
            handle.edDeseired.Limits = [ handle.range.required.min handle.range.required.max];
            handle.edDeseired.ValueDisplayFormat = '%2.2f';


             x = x + handle.number_w + handle.margin_w;
            
            % Create sliderDeseired
            handle.sliderDeseired = uislider(handle.board);
            handle.sliderDeseired.MajorTicks = [];
            handle.sliderDeseired.MajorTickLabels = {''};
            handle.sliderDeseired.ValueChangingFcn = @handle.sliderDeseiredChanging;
            handle.sliderDeseired.MinorTicks = [];
            handle.sliderDeseired.Limits = [ handle.range.required.min handle.range.required.max];
            gain= handle.slider_w/(handle.range.actual.max- handle.range.actual.min)
            slider_w= gain * (handle.range.required.max- handle.range.required.min) ;
            ofset_x=gain * (handle.range.required.min - handle.range.actual.min)
            handle.sliderDeseired.Position = [x+10+ofset_x y+round(handle.text_h/2) slider_w-20 3];

            x = handle.margin_w + handle.marker_w + handle.margin_w;
            y= handle.board_h - 2 * handle.margin_h - 2 * handle.text_h;
            
                        % Create Label
            handle.lblActual = uilabel(handle.board);
            handle.lblActual.HorizontalAlignment = 'right';
            handle.lblActual.Position = [x y handle.number_w handle.text_h];

            x = x + handle.number_w + handle.margin_w;

            % Create Gauge
            handle.gaugeActual = uigauge(handle.board, 'linear');
            handle.gaugeActual.Position = [x y  handle.slider_w handle.text_h+round(handle.text_h/2)];  
            handle.gaugeActual.Limits =  [ handle.range.actual.min handle.range.actual.max];
         
            handle.setRequired(coord.actual);
            handle.setActual(coord.actual);

        end
        function delete(handle)
            delete(handle.board);
        end
   end
   
    methods (Access = private)

        function sliderDeseiredChanging(hObject, eventdata, handles, varargin)
                hObject.setRequired(handles.Value)
        end

        % Value changed function: ZEditField
        function edDeseiredChanged( hObject, eventdata, handles, varargin)
                hObject.setRequired(handles.Value)
        end
        
        function edDeseiredMouseWheelMovedCallback( hObject, eventdata, handles, varargin)
        end

    end
end