set PROTOC_HOME="C:\Work\dev\nanopb-0.4.5-windows-x86"

set PROTO=message

%PROTOC_HOME%\generator-bin\protoc.exe --nanopb_out=src %PROTO%.proto