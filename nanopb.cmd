set PROTOC_HOME="C:\Dev\nanopb-0.4.3-windows-x86"

set PROTO=message

%PROTOC_HOME%\generator-bin\protoc.exe --nanopb_out=src %PROTO%.proto