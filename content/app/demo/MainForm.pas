unit MainForm;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, sSkinProvider, sSkinManager, ComCtrls, ToolWin, sToolBar,
  ImgList, acAlphaImageList, ActnList;

type
  TForm1 = class(TForm)
    sSkinManager1: TsSkinManager;
    sSkinProvider1: TsSkinProvider;
    sToolBar1: TsToolBar;
    ToolButton1: TToolButton;
    ToolButton2: TToolButton;
    ActionList1: TActionList;
    aStop: TAction;
    aStart: TAction;
    sAlphaImageList1: TsAlphaImageList;
    ToolButton3: TToolButton;
    aCalibrate: TAction;
    procedure aStartExecute(Sender: TObject);
    procedure aCalibrateExecute(Sender: TObject);
  private
    { Private declarations }
  public
    { Public declarations }
  end;

var
  Form1: TForm1;

implementation

{$R *.dfm}

var
  path:String;
  ComFile:THandle = INVALID_HANDLE_VALUE ;
  Dcb:TDCB;
  ComStat:TComStat;
  Timeouts:TCommTimeouts;
  OverRead:TOverlapped;
  Adr:PChar;
  Buffer:array [0..255] of AnsiChar;

function CHeckHandle :Boolean;
begin
  if ComFile = INVALID_HANDLE_VALUE  then begin
    Result :=  False;
    path:='\\.\COM11';
    Adr:=PChar(path);
    ComFile:=CreateFile(Adr, Generic_Read+Generic_Write, 0, nil,
    Open_Existing, File_Flag_Overlapped, 0);
    if ComFile=INVALID_HANDLE_VALUE   then
    begin
      ShowMessage('FUCK - 1!');
      exit;
    end;

    PurgeComm(ComFile, Purge_TXabort or Purge_RXabort or Purge_TXclear or Purge_RXclear);

    GetCommState(ComFile, DCB);;

    with DCB do
    begin
      BaudRate:=115200;
      ByteSize:=8;
      Parity:=NoParity;
      StopBits:=OneStopBit;
    end;

    if not SetCommState(ComFile, DCB) then
    begin
      ShowMessage('FUCK - 2');
      CloseHandle(ComFile);
      exit;
    end;
 {
    GetCommTimeouts(ComFile, Timeouts); // Чтение текущих таймаутов и настройка параметров структуры CommTimeouts
    Timeouts.ReadIntervalTimeout:=MAXDWORD; //Таймаут между двумя символами;
    Timeouts.ReadTotalTimeoutMultiplier:=0; //Общий таймаут операции чтения;
    Timeouts.ReadTotalTimeoutConstant:=0; //Константа для общего таймаута операции чтения;
    Timeouts.WriteTotalTimeoutMultiplier:=0; //Общий таймаут операции записи;
    Timeouts.WriteTotalTimeoutConstant:=0; //Константа для общего таймаута операции записи;
    SetCommTimeouts(ComFile, Timeouts); //Установка таймаутов;

    SetupComm(ComFile, 4096, 4096); //Настройка буферов;
    if not SetupComm(ComFile, 4096, 4096) then //Ошибка настройки буферов;
    begin
      ShowMessage('Ошибка настройки буферов');
      CloseHandle(ComFile);
      exit;
    end;
    *}

//    SetCommMask(ComFile, EV_RXchar); {Устанавливаем маску для срабатывания по событию - "Прием байта в порт"}

    Result :=  True;
  end else begin
    Result :=  True;
  end;
end;

procedure TForm1.aStartExecute(Sender: TObject);
var
  str:String;
  writed : CArdinal;
  buf:PAnsiChar;
begin
  if CHeckHandle then begin
    str:='{"command":"STOP"}\n';
    buf := PAnsiChar(str);
    WriteFile(ComFile,buf,Length(str),writed, nil) ;
  end;
end;

procedure TForm1.aCalibrateExecute(Sender: TObject);
var
  str:String;
  writed : CArdinal;
  buf:PAnsiChar;
begin
  if CHeckHandle then begin
    str:='{"command":"CALIBRATE"}\n';
    buf := PAnsiChar(str);
    WriteFile(ComFile,buf,Length(str),writed, nil) ;
  end;
end;

end.
