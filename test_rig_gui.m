function varargout = test_rig_gui(varargin)
% TEST_RIG_GUI MATLAB code for test_rig_gui.fig
%      TEST_RIG_GUI, by itself, creates a new TEST_RIG_GUI or raises the existing
%      singleton*.
%
%      H = TEST_RIG_GUI returns the handle to a new TEST_RIG_GUI or the handle to
%      the existing singleton*.
%
%      TEST_RIG_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEST_RIG_GUI.M with the given input arguments.
%
%      TEST_RIG_GUI('Property','Value',...) creates a new TEST_RIG_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before test_rig_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to test_rig_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help test_rig_gui

% Last Modified by GUIDE v2.5 02-Jan-2019 12:54:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @test_rig_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @test_rig_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before test_rig_gui is made visible.
function test_rig_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to test_rig_gui (see VARARGIN)

% Choose default command line output for test_rig_gui
handles.output = hObject;

ser = establish_connection(); 
%% TODO -- check that the serial connection returned is open (and therefor valid)
handles.serial = ser;



% Update handles structure
guidata(hObject, handles);


% UIWAIT makes test_rig_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = test_rig_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pb_forward.
function pb_forward_Callback(hObject, eventdata, handles)
% hObject    handle to pb_forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
jog_dist = str2double(get(handles.jog_dist, 'string'));
max_jog = 50; 
if jog_dist > max_jog        % make 
    set(handles.jog_dist, 'string', max_jog);
    jog_dist = max_jog;
elseif jog_dist < 1
    set(handles.jog_dist, 'string', 1);
    jog_dist = 1;
end 
ser = handles.serial; 
fprintf('Jogging Forward: %d\n', jog_dist);
jog(ser, jog_dist); 



% --- Executes on button press in pb_backward.
function pb_backward_Callback(hObject, eventdata, handles)
% hObject    handle to pb_backward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
jog_dist = str2double(get(handles.jog_dist, 'string'));
max_jog = 50; 
if jog_dist > max_jog        % make 
    set(handles.jog_dist, 'string', max_jog);
    jog_dist = max_jog;
elseif jog_dist < 1
    set(handles.jog_dist, 'string', 1);
    jog_dist = 1;
end 
ser = handles.serial; 
fprintf('Jogging Backward: %d\n', jog_dist);
jog(ser, -jog_dist); 

% --- Executes on button press in pb_start.
function pb_start_Callback(hObject, eventdata, handles)
% hObject    handle to pb_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

output_file = get(handles.file_select, 'string');


original_text = get(handles.pb_start, 'string'); 
original_color = get(handles.pb_start, 'BackgroundColor'); 


speed = str2double(get(handles.test_speed, 'string'));
distance = str2double(get(handles.test_dist, 'string'));
cycles = str2double(get(handles.cycles, 'string'));


set(handles.pb_start, 'string',...
                        sprintf('Running Test %d of %d...', 1, cycles)); 
set(handles.pb_start, 'BackgroundColor', [0.8, 0.8, 0.8]); pause(0.1); 

fig = handles.figure1; 

test_data = run_test(handles.serial, distance, speed, fig);

if cycles > 1
    for i = 2:cycles
        set(handles.pb_start, 'string', ...
                         sprintf('Running Test %d of %d...', i, cycles)); 
        pause(0.1); 
        test_data = [test_data; 0, 0, 0];   % add empty row between tests
        next_test = run_test(handles.serial, distance, speed, fig);
        test_data = [test_data; next_test];
    end 
end 

T = table(test_data(:, 1), test_data(:, 2), test_data(:, 3),...
                  'VariableNames', {'Time', 'Displacement', 'Data'});

writetable(T, output_file);                    
fprintf('Writing test data to: %s\n', output_file); 

set(handles.pb_start, 'string', original_text); 
set(handles.pb_start, 'BackgroundColor', original_color); pause(0.1); 


function test_speed_Callback(hObject, eventdata, handles)
% hObject    handle to test_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of test_speed as text
%        str2double(get(hObject,'String')) returns contents of test_speed as a double


% --- Executes during object creation, after setting all properties.
function test_speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to test_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function jog_dist_Callback(hObject, eventdata, handles)
% hObject    handle to jog_dist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of jog_dist as text
%        str2double(get(hObject,'String')) returns contents of jog_dist as a double


% --- Executes during object creation, after setting all properties.
function jog_dist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to jog_dist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function test_dist_Callback(hObject, eventdata, handles)
% hObject    handle to test_dist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of test_dist as text
%        str2double(get(hObject,'String')) returns contents of test_dist as a double


% --- Executes during object creation, after setting all properties.
function test_dist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to test_dist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function cycles_Callback(hObject, eventdata, handles)
% hObject    handle to cycles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cycles as text
%        str2double(get(hObject,'String')) returns contents of cycles as a double


% --- Executes during object creation, after setting all properties.
function cycles_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cycles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_home.
function pb_home_Callback(hObject, eventdata, handles)
% hObject    handle to pb_home (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
home(handles.serial);




function file_select_Callback(hObject, eventdata, handles)
% hObject    handle to file_select (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of file_select as text
%        str2double(get(hObject,'String')) returns contents of file_select as a double
%input_folder_name = uigetdir; % maybe add string output in log 
[input_file_name, path] = uiputfile('*.csv'); % maybe add string output in log 
set(handles.file_select, 'string', fullfile(path, input_file_name));





%% =============================================================================
%
%                               Non-GUI Functions 
%
% ==============================================================================

    
function ser = establish_connection()
%%   
%  Establishes connectino to arduino board
%
%   ser - serial connection object, will be open if connection is established 
%


% https://www.mathworks.com/matlabcentral/answers/322930-why-are-my-serial-ports-unable-to-open-with-matlab
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Establishing Connection with Arduino Board...\n'); 

sl = seriallist; 
connected = false; 
k = 3; 
ns = length(sl);

while (~connected) && (k <= ns)
	fprintf('Trying port: %s\n', sl{k}); 
	ser = serial(sl{k}, 'BaudRate', 115200); % for now -- could make more robust 

	try 
		fopen(ser); 
		pause(2)

		command = hex2dec('A6');	% ask the device to identify itself 
		fwrite(ser, command); 
		fwrite(ser, command); 
		pause(1);		           % give the arduino a chance to respond 
		if (get(ser,'BytesAvailable') > 0)
			check = fread(ser, 1); 
			if (check == hex2dec('55'))
				connected = true;
				fprintf(['Connection established to Arduino',...
                                            ' on port: %s\n'], sl{k}); 
			end 	
		else
			fclose(ser);
		end 

	catch ME
		% well, move on to the next one 
	end 
	k = k + 1; 
end 
pause(2); 




function jog(ser, dist)
%%
%   J
%
%
%	ser - serial communication object 
%	dist - distance in 1 mm increments 
%
jog_command = hex2dec('A0'); 

% send 128 for zero 

if dist > 0 
    val_out = 128 + dist;
else 
    val_out = 127 + dist; 
end 

fwrite(ser, jog_command); 
fwrite(ser, val_out); 
pause(0.01); 



function home(ser)
%%
%
%
%


fprintf('Homing Platform...\n');
home_command = hex2dec('A2'); 
fwrite(ser, home_command); 
fwrite(ser, 0); 
pause(0.01); 




function test_data = run_test(ser, distance, speed, fig)
%%
%	
%
%
%	ser - serial communication object 
%	distance - distance in mm to travel in test
%	speed - speed in mm/s for translation of platform 


cla reset;                  % clear data from the figure 
al = animatedline('Color', 'r');          % define animated line
al2 = animatedline('Color', 'b');          % define animated line

%% TODO -- add some plot axis formating

pitch = 2; 					    % mm/thread 
steps_per_rev = 200; 		    % 4 from quarter stepping 
rps = speed/pitch;  			% rev per second 
sps = rps * steps_per_rev; 	    % steps per second 
step_delay = 1/sps; 			% in seconds 


revs = distance/pitch; 
total_steps = steps_per_rev * revs;

% For debug 
fprintf('Running Test...\n');
fprintf('\t Total steps: %d\n', total_steps);
fprintf('\t Step Delay steps: %0.3f microseconds\n', 1e6 * step_delay);


test_command = hex2dec('A4');        % command byte to run a test 
% multiply distance sent by 10 to give us more resolution -- will divide by
% 10 on the arduino side to account for this 
distance_16bit = cast(10 * distance, 'uint16'); 	
step_delay_16bit = cast(1e6 * step_delay, 'uint16');

% Total bytes to write: 5
fwrite(ser, test_command);                      % test command byte                          

fwrite(ser, distance_16bit);                    % low byte
fwrite(ser, bitshift(distance_16bit, -8));      % high byte		

fwrite(ser, step_delay_16bit);                  % low byte
fwrite(ser, bitshift(step_delay_16bit, -8));    % high byte	


%% Read the serial from Arduino, display to monitor, plot, and write to file 
% -- byte 0-3 - time since test began in microseconds 
% -- byte 4-5 - steps taken since test began 
% -- byte 6-7 - data!

test_data = zeros(1e6, 3);  % allocate more than enough rows for data 
fprintf('Wating for serial response...\n');
while (get(ser,'BytesAvailable') <= 0)	 % wait for serial to become available 
end 


% now serial has become available wait for end of test time flag!
time = 0; prev_steps = 0; 
max32bit = 2^32 - 1;
count = 0;      % use this so we dont update the plot at every single point

while(time ~= max32bit)
    % disp(get(ser,'BytesAvailable')); 

    if (get(ser,'BytesAvailable') >= 8)
        time = double(typecast(uint8(fread(ser, 4)), 'uint32'));
        steps = double(typecast(uint8(fread(ser, 2)), 'uint16'));
        data = double(typecast(uint8(fread(ser, 2)), 'uint16')); 
                
        if (time ~= max32bit)

            count = count + 1; 
            dst = pitch * steps/steps_per_rev;

            test_data(count, :) = [time, dst, data]; 
            fprintf('Time: %0.4f, \tDistance: %0.3f mm, \tData: %d\n',...
                                                             time, dst, data);
            %addpoints(al, dst, data);
            
            if (steps >= prev_steps)
                addpoints(al, dst, data);
            else
                addpoints(al2, dst, data);
            end
            
            prev_steps = steps; 
            if (mod(count, 10) == 0)
                drawnow;
            end 
        end 
    end 
end 

% truncate off the zeros remaining at end of test data 
test_data = test_data(1:count, :); 
