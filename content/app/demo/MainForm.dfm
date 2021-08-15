object Form1: TForm1
  Left = 345
  Top = 163
  Width = 959
  Height = 675
  Caption = #1040#1074#1072#1090#1072#1088'-'#1076#1077#1084#1086
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -19
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 24
  object sToolBar1: TsToolBar
    Left = 0
    Top = 0
    Width = 943
    Height = 36
    AutoSize = True
    ButtonHeight = 32
    ButtonWidth = 113
    Caption = 'sToolBar1'
    ShowCaptions = True
    TabOrder = 0
    SkinData.SkinSection = 'TOOLBAR'
    object ToolButton1: TToolButton
      Left = 0
      Top = 2
      Action = aStart
    end
    object ToolButton2: TToolButton
      Left = 113
      Top = 2
      Action = aStop
    end
    object ToolButton3: TToolButton
      Left = 226
      Top = 2
      Action = aCalibrate
    end
  end
  object sSkinManager1: TsSkinManager
    InternalSkins = <>
    MenuSupport.ExtraLineFont.Charset = DEFAULT_CHARSET
    MenuSupport.ExtraLineFont.Color = clWindowText
    MenuSupport.ExtraLineFont.Height = -11
    MenuSupport.ExtraLineFont.Name = 'MS Sans Serif'
    MenuSupport.ExtraLineFont.Style = []
    SkinDirectory = 'C:\Skins'
    SkinName = 'DarkMetro'
    SkinInfo = '8.12'
    ThirdParty.ThirdEdits = ' '
    ThirdParty.ThirdButtons = 'TButton'
    ThirdParty.ThirdBitBtns = ' '
    ThirdParty.ThirdCheckBoxes = ' '
    ThirdParty.ThirdGroupBoxes = ' '
    ThirdParty.ThirdListViews = ' '
    ThirdParty.ThirdPanels = ' '
    ThirdParty.ThirdGrids = ' '
    ThirdParty.ThirdTreeViews = ' '
    ThirdParty.ThirdComboBoxes = ' '
    ThirdParty.ThirdWWEdits = ' '
    ThirdParty.ThirdVirtualTrees = ' '
    ThirdParty.ThirdGridEh = ' '
    ThirdParty.ThirdPageControl = ' '
    ThirdParty.ThirdTabControl = ' '
    ThirdParty.ThirdToolBar = ' '
    ThirdParty.ThirdStatusBar = ' '
    ThirdParty.ThirdSpeedButton = ' '
    ThirdParty.ThirdScrollControl = ' '
    ThirdParty.ThirdUpDown = ' '
    ThirdParty.ThirdScrollBar = ' '
    Left = 752
    Top = 80
  end
  object sSkinProvider1: TsSkinProvider
    AddedTitle.Font.Charset = DEFAULT_CHARSET
    AddedTitle.Font.Color = clNone
    AddedTitle.Font.Height = -11
    AddedTitle.Font.Name = 'MS Sans Serif'
    AddedTitle.Font.Style = []
    SkinData.SkinSection = 'FORM'
    TitleButtons = <>
    Left = 784
    Top = 80
  end
  object ActionList1: TActionList
    Left = 824
    Top = 80
    object aStop: TAction
      Caption = #1057#1090#1086#1087
    end
    object aStart: TAction
      Caption = #1057#1090#1072#1088#1090
      OnExecute = aStartExecute
    end
    object aCalibrate: TAction
      Caption = #1050#1072#1083#1080#1073#1088#1086#1074#1082#1072
      OnExecute = aCalibrateExecute
    end
  end
  object sAlphaImageList1: TsAlphaImageList
    Height = 32
    Width = 32
    Items = <>
    Left = 872
    Top = 80
  end
end
