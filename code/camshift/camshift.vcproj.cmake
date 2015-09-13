<?xml version="1.0" encoding = "Windows-1252"?>
<VisualStudioProject
	ProjectType="Visual C++"
	Version="8.00"
	Name="camshift"
	ProjectGUID="{BC56FBF8-9B74-4841-BA49-7091FB2A8E33}"
	SccProjectName=""
	SccLocalPath=""
	Keyword="Win32Proj">
	<Platforms>
		<Platform
			Name="Win32"/>
	</Platforms>
	<Configurations>
		<Configuration
			Name="Debug|Win32"
			OutputDirectory="Debug"
			IntermediateDirectory="camshift.dir\Debug"
			ConfigurationType="1"
			UseOfMFC="0"
			ATLMinimizesCRunTimeLibraryUsage="FALSE"
			CharacterSet="2">
			<Tool
				Name="VCCLCompilerTool"
				AdditionalOptions=" /Zm1000"
				AdditionalIncludeDirectories="C:\ACE-5.6\ACE_wrappers;C:\yarp2\src\libYARP_OS\include;C:\yarp2\src\libYARP_sig\include;C:\yarp2\src\libYARP_dev\include;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift&quot;;&quot;C:\Program Files\OpenCV\cv\include&quot;;&quot;C:\Program Files\OpenCV\cvaux\include&quot;;&quot;C:\Program Files\OpenCV\cv\include\..&quot;;&quot;C:\Program Files\OpenCV\cxcore\include&quot;;&quot;C:\Program Files\OpenCV\otherlibs\highgui&quot;;"
				BasicRuntimeChecks="3"
				CompileAs="2"
				DebugInformationFormat="3"
				ExceptionHandling="1"
				InlineFunctionExpansion="0"
				Optimization="0"
				RuntimeLibrary="3"
				RuntimeTypeInfo="TRUE"
				WarningLevel="3"
				PreprocessorDefinitions="WIN32,_WINDOWS,_DEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;Debug\&quot;&quot;"
				AssemblerListingLocation="Debug"
				ObjectFile="$(IntDir)\"
				ProgramDataBaseFileName="C:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift/Debug/camshift.pdb"
/>
			<Tool
				Name="VCCustomBuildTool"/>
			<Tool
				Name="VCResourceCompilerTool"
				AdditionalIncludeDirectories="C:\ACE-5.6\ACE_wrappers;C:\yarp2\src\libYARP_OS\include;C:\yarp2\src\libYARP_sig\include;C:\yarp2\src\libYARP_dev\include;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift&quot;;&quot;C:\Program Files\OpenCV\cv\include&quot;;&quot;C:\Program Files\OpenCV\cvaux\include&quot;;&quot;C:\Program Files\OpenCV\cv\include\..&quot;;&quot;C:\Program Files\OpenCV\cxcore\include&quot;;&quot;C:\Program Files\OpenCV\otherlibs\highgui&quot;;"
				PreprocessorDefinitions="WIN32,_WINDOWS,_DEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;Debug\&quot;&quot;"/>
			<Tool
				Name="VCMIDLTool"
				PreprocessorDefinitions="WIN32,_WINDOWS,_DEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;Debug\&quot;&quot;"
				MkTypLibCompatible="FALSE"
				TargetEnvironment="1"
				GenerateStublessProxies="TRUE"
				TypeLibraryName="$(InputName).tlb"
				OutputDirectory="$(IntDir)"
				HeaderFileName="$(InputName).h"
				DLLDataFileName=""
				InterfaceIdentifierFileName="$(InputName)_i.c"
				ProxyFileName="$(InputName)_p.c"/>
			<Tool
				Name="VCManifestTool"
				UseFAT32Workaround="true"
			/>
			<Tool
				Name="VCPreBuildEventTool"/>
			<Tool
				Name="VCPreLinkEventTool"/>
			<Tool
				Name="VCPostBuildEventTool"/>
			<Tool
				Name="VCLinkerTool"
				AdditionalOptions=" /STACK:10000000 /machine:I386 /debug"
				AdditionalDependencies="$(NOINHERIT) kernel32.lib user32.lib gdi32.lib winspool.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comdlg32.lib advapi32.lib  YARP_devd.lib YARP_sigd.lib YARP_OSd.lib winmm.lib ACEd.lib yarpmodd.lib YARP_devd.lib YARP_sigd.lib YARP_OSd.lib winmm.lib ACEd.lib cv.lib cvaux.lib cxcore.lib highgui.lib PGRFlyCapture.lib FlexMS32.lib "
				OutputFile="Debug\camshift.exe"
				Version="0.0"
				GenerateManifest="TRUE"
				LinkIncremental="2"
				AdditionalLibraryDirectories="c:\yarp2\lib\$(OutDir)\$(OutDir),c:\yarp2\lib\$(OutDir),c:\yarp2\lib\$(OutDir)\Debug\$(OutDir),c:\yarp2\lib\$(OutDir)\Debug,c:\yarp2\lib\$(OutDir)\Release\$(OutDir),c:\yarp2\lib\$(OutDir)\Release,C:\ACE-5.6\ACE_wrappers\lib\$(OutDir),C:\ACE-5.6\ACE_wrappers\lib,&quot;C:\Program Files\OpenCV\lib\$(OutDir)&quot;,&quot;C:\Program Files\OpenCV\lib&quot;,C:\iCub\src\modules\dragonflyApi\winnt\lib\$(OutDir),C:\iCub\src\modules\dragonflyApi\winnt\lib,C:\vislab\dev\niflex\winnt\dd_orig\lib\$(OutDir),C:\vislab\dev\niflex\winnt\dd_orig\lib"
				ProgramDataBaseFile="$(OutDir)\camshift.pdb"
				GenerateDebugInformation="TRUE"
				SubSystem="1"
				StackReserveSize="10000000"/>
		</Configuration>
		<Configuration
			Name="Release|Win32"
			OutputDirectory="Release"
			IntermediateDirectory="camshift.dir\Release"
			ConfigurationType="1"
			UseOfMFC="0"
			ATLMinimizesCRunTimeLibraryUsage="FALSE"
			CharacterSet="2">
			<Tool
				Name="VCCLCompilerTool"
				AdditionalOptions=" /Zm1000"
				AdditionalIncludeDirectories="C:\ACE-5.6\ACE_wrappers;C:\yarp2\src\libYARP_OS\include;C:\yarp2\src\libYARP_sig\include;C:\yarp2\src\libYARP_dev\include;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift&quot;;&quot;C:\Program Files\OpenCV\cv\include&quot;;&quot;C:\Program Files\OpenCV\cvaux\include&quot;;&quot;C:\Program Files\OpenCV\cv\include\..&quot;;&quot;C:\Program Files\OpenCV\cxcore\include&quot;;&quot;C:\Program Files\OpenCV\otherlibs\highgui&quot;;"
				CompileAs="2"
				ExceptionHandling="1"
				InlineFunctionExpansion="2"
				Optimization="2"
				RuntimeLibrary="2"
				RuntimeTypeInfo="TRUE"
				WarningLevel="3"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;Release\&quot;&quot;"
				AssemblerListingLocation="Release"
				ObjectFile="$(IntDir)\"
				ProgramDataBaseFileName="C:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift/Release/camshift.pdb"
/>
			<Tool
				Name="VCCustomBuildTool"/>
			<Tool
				Name="VCResourceCompilerTool"
				AdditionalIncludeDirectories="C:\ACE-5.6\ACE_wrappers;C:\yarp2\src\libYARP_OS\include;C:\yarp2\src\libYARP_sig\include;C:\yarp2\src\libYARP_dev\include;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift&quot;;&quot;C:\Program Files\OpenCV\cv\include&quot;;&quot;C:\Program Files\OpenCV\cvaux\include&quot;;&quot;C:\Program Files\OpenCV\cv\include\..&quot;;&quot;C:\Program Files\OpenCV\cxcore\include&quot;;&quot;C:\Program Files\OpenCV\otherlibs\highgui&quot;;"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;Release\&quot;&quot;"/>
			<Tool
				Name="VCMIDLTool"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;Release\&quot;&quot;"
				MkTypLibCompatible="FALSE"
				TargetEnvironment="1"
				GenerateStublessProxies="TRUE"
				TypeLibraryName="$(InputName).tlb"
				OutputDirectory="$(IntDir)"
				HeaderFileName="$(InputName).h"
				DLLDataFileName=""
				InterfaceIdentifierFileName="$(InputName)_i.c"
				ProxyFileName="$(InputName)_p.c"/>
			<Tool
				Name="VCManifestTool"
				UseFAT32Workaround="true"
			/>
			<Tool
				Name="VCPreBuildEventTool"/>
			<Tool
				Name="VCPreLinkEventTool"/>
			<Tool
				Name="VCPostBuildEventTool"/>
			<Tool
				Name="VCLinkerTool"
				AdditionalOptions=" /STACK:10000000 /machine:I386"
				AdditionalDependencies="$(NOINHERIT) kernel32.lib user32.lib gdi32.lib winspool.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comdlg32.lib advapi32.lib  YARP_dev.lib YARP_sig.lib YARP_OS.lib ACE.lib winmm.lib yarpmod.lib YARP_dev.lib YARP_sig.lib YARP_OS.lib ACE.lib winmm.lib cv.lib cvaux.lib cxcore.lib highgui.lib PGRFlyCapture.lib FlexMS32.lib "
				OutputFile="Release\camshift.exe"
				Version="0.0"
				GenerateManifest="TRUE"
				LinkIncremental="1"
				AdditionalLibraryDirectories="c:\yarp2\lib\$(OutDir)\$(OutDir),c:\yarp2\lib\$(OutDir),c:\yarp2\lib\$(OutDir)\Debug\$(OutDir),c:\yarp2\lib\$(OutDir)\Debug,c:\yarp2\lib\$(OutDir)\Release\$(OutDir),c:\yarp2\lib\$(OutDir)\Release,C:\ACE-5.6\ACE_wrappers\lib\$(OutDir),C:\ACE-5.6\ACE_wrappers\lib,&quot;C:\Program Files\OpenCV\lib\$(OutDir)&quot;,&quot;C:\Program Files\OpenCV\lib&quot;,C:\iCub\src\modules\dragonflyApi\winnt\lib\$(OutDir),C:\iCub\src\modules\dragonflyApi\winnt\lib,C:\vislab\dev\niflex\winnt\dd_orig\lib\$(OutDir),C:\vislab\dev\niflex\winnt\dd_orig\lib"
				ProgramDataBaseFile="$(OutDir)\camshift.pdb"
				SubSystem="1"
				StackReserveSize="10000000"/>
		</Configuration>
		<Configuration
			Name="MinSizeRel|Win32"
			OutputDirectory="MinSizeRel"
			IntermediateDirectory="camshift.dir\MinSizeRel"
			ConfigurationType="1"
			UseOfMFC="0"
			ATLMinimizesCRunTimeLibraryUsage="FALSE"
			CharacterSet="2">
			<Tool
				Name="VCCLCompilerTool"
				AdditionalOptions=" /Zm1000"
				AdditionalIncludeDirectories="C:\ACE-5.6\ACE_wrappers;C:\yarp2\src\libYARP_OS\include;C:\yarp2\src\libYARP_sig\include;C:\yarp2\src\libYARP_dev\include;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift&quot;;&quot;C:\Program Files\OpenCV\cv\include&quot;;&quot;C:\Program Files\OpenCV\cvaux\include&quot;;&quot;C:\Program Files\OpenCV\cv\include\..&quot;;&quot;C:\Program Files\OpenCV\cxcore\include&quot;;&quot;C:\Program Files\OpenCV\otherlibs\highgui&quot;;"
				CompileAs="2"
				ExceptionHandling="1"
				InlineFunctionExpansion="1"
				Optimization="1"
				RuntimeLibrary="2"
				RuntimeTypeInfo="TRUE"
				WarningLevel="3"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;MinSizeRel\&quot;&quot;"
				AssemblerListingLocation="MinSizeRel"
				ObjectFile="$(IntDir)\"
				ProgramDataBaseFileName="C:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift/MinSizeRel/camshift.pdb"
/>
			<Tool
				Name="VCCustomBuildTool"/>
			<Tool
				Name="VCResourceCompilerTool"
				AdditionalIncludeDirectories="C:\ACE-5.6\ACE_wrappers;C:\yarp2\src\libYARP_OS\include;C:\yarp2\src\libYARP_sig\include;C:\yarp2\src\libYARP_dev\include;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift&quot;;&quot;C:\Program Files\OpenCV\cv\include&quot;;&quot;C:\Program Files\OpenCV\cvaux\include&quot;;&quot;C:\Program Files\OpenCV\cv\include\..&quot;;&quot;C:\Program Files\OpenCV\cxcore\include&quot;;&quot;C:\Program Files\OpenCV\otherlibs\highgui&quot;;"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;MinSizeRel\&quot;&quot;"/>
			<Tool
				Name="VCMIDLTool"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;MinSizeRel\&quot;&quot;"
				MkTypLibCompatible="FALSE"
				TargetEnvironment="1"
				GenerateStublessProxies="TRUE"
				TypeLibraryName="$(InputName).tlb"
				OutputDirectory="$(IntDir)"
				HeaderFileName="$(InputName).h"
				DLLDataFileName=""
				InterfaceIdentifierFileName="$(InputName)_i.c"
				ProxyFileName="$(InputName)_p.c"/>
			<Tool
				Name="VCManifestTool"
				UseFAT32Workaround="true"
			/>
			<Tool
				Name="VCPreBuildEventTool"/>
			<Tool
				Name="VCPreLinkEventTool"/>
			<Tool
				Name="VCPostBuildEventTool"/>
			<Tool
				Name="VCLinkerTool"
				AdditionalOptions=" /STACK:10000000 /machine:I386"
				AdditionalDependencies="$(NOINHERIT) kernel32.lib user32.lib gdi32.lib winspool.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comdlg32.lib advapi32.lib  YARP_dev.lib YARP_sig.lib YARP_OS.lib ACE.lib winmm.lib yarpmod.lib YARP_dev.lib YARP_sig.lib YARP_OS.lib ACE.lib winmm.lib cv.lib cvaux.lib cxcore.lib highgui.lib PGRFlyCapture.lib FlexMS32.lib "
				OutputFile="MinSizeRel\camshift.exe"
				Version="0.0"
				GenerateManifest="TRUE"
				LinkIncremental="1"
				AdditionalLibraryDirectories="c:\yarp2\lib\$(OutDir)\$(OutDir),c:\yarp2\lib\$(OutDir),c:\yarp2\lib\$(OutDir)\Debug\$(OutDir),c:\yarp2\lib\$(OutDir)\Debug,c:\yarp2\lib\$(OutDir)\Release\$(OutDir),c:\yarp2\lib\$(OutDir)\Release,C:\ACE-5.6\ACE_wrappers\lib\$(OutDir),C:\ACE-5.6\ACE_wrappers\lib,&quot;C:\Program Files\OpenCV\lib\$(OutDir)&quot;,&quot;C:\Program Files\OpenCV\lib&quot;,C:\iCub\src\modules\dragonflyApi\winnt\lib\$(OutDir),C:\iCub\src\modules\dragonflyApi\winnt\lib,C:\vislab\dev\niflex\winnt\dd_orig\lib\$(OutDir),C:\vislab\dev\niflex\winnt\dd_orig\lib"
				ProgramDataBaseFile="$(OutDir)\camshift.pdb"
				SubSystem="1"
				StackReserveSize="10000000"/>
		</Configuration>
		<Configuration
			Name="RelWithDebInfo|Win32"
			OutputDirectory="RelWithDebInfo"
			IntermediateDirectory="camshift.dir\RelWithDebInfo"
			ConfigurationType="1"
			UseOfMFC="0"
			ATLMinimizesCRunTimeLibraryUsage="FALSE"
			CharacterSet="2">
			<Tool
				Name="VCCLCompilerTool"
				AdditionalOptions=" /Zm1000"
				AdditionalIncludeDirectories="C:\ACE-5.6\ACE_wrappers;C:\yarp2\src\libYARP_OS\include;C:\yarp2\src\libYARP_sig\include;C:\yarp2\src\libYARP_dev\include;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift&quot;;&quot;C:\Program Files\OpenCV\cv\include&quot;;&quot;C:\Program Files\OpenCV\cvaux\include&quot;;&quot;C:\Program Files\OpenCV\cv\include\..&quot;;&quot;C:\Program Files\OpenCV\cxcore\include&quot;;&quot;C:\Program Files\OpenCV\otherlibs\highgui&quot;;"
				CompileAs="2"
				DebugInformationFormat="3"
				ExceptionHandling="1"
				InlineFunctionExpansion="1"
				Optimization="2"
				RuntimeLibrary="2"
				RuntimeTypeInfo="TRUE"
				WarningLevel="3"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;RelWithDebInfo\&quot;&quot;"
				AssemblerListingLocation="RelWithDebInfo"
				ObjectFile="$(IntDir)\"
				ProgramDataBaseFileName="C:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift/RelWithDebInfo/camshift.pdb"
/>
			<Tool
				Name="VCCustomBuildTool"/>
			<Tool
				Name="VCResourceCompilerTool"
				AdditionalIncludeDirectories="C:\ACE-5.6\ACE_wrappers;C:\yarp2\src\libYARP_OS\include;C:\yarp2\src\libYARP_sig\include;C:\yarp2\src\libYARP_dev\include;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift&quot;;&quot;C:\Program Files\OpenCV\cv\include&quot;;&quot;C:\Program Files\OpenCV\cvaux\include&quot;;&quot;C:\Program Files\OpenCV\cv\include\..&quot;;&quot;C:\Program Files\OpenCV\cxcore\include&quot;;&quot;C:\Program Files\OpenCV\otherlibs\highgui&quot;;"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;RelWithDebInfo\&quot;&quot;"/>
			<Tool
				Name="VCMIDLTool"
				PreprocessorDefinitions="WIN32,_WINDOWS,NDEBUG,_REENTRANT,WIN32,_WINDOWS,YARP_LITTLE_ENDIAN,&quot;YARP_INT32=int&quot;,&quot;YARP_INT16=short&quot;,&quot;YARP_FLOAT64=double&quot;,&quot;CMAKE_INTDIR=\&quot;RelWithDebInfo\&quot;&quot;"
				MkTypLibCompatible="FALSE"
				TargetEnvironment="1"
				GenerateStublessProxies="TRUE"
				TypeLibraryName="$(InputName).tlb"
				OutputDirectory="$(IntDir)"
				HeaderFileName="$(InputName).h"
				DLLDataFileName=""
				InterfaceIdentifierFileName="$(InputName)_i.c"
				ProxyFileName="$(InputName)_p.c"/>
			<Tool
				Name="VCManifestTool"
				UseFAT32Workaround="true"
			/>
			<Tool
				Name="VCPreBuildEventTool"/>
			<Tool
				Name="VCPreLinkEventTool"/>
			<Tool
				Name="VCPostBuildEventTool"/>
			<Tool
				Name="VCLinkerTool"
				AdditionalOptions=" /STACK:10000000 /machine:I386 /debug"
				AdditionalDependencies="$(NOINHERIT) kernel32.lib user32.lib gdi32.lib winspool.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comdlg32.lib advapi32.lib  YARP_dev.lib YARP_sig.lib YARP_OS.lib ACE.lib winmm.lib yarpmod.lib YARP_dev.lib YARP_sig.lib YARP_OS.lib ACE.lib winmm.lib cv.lib cvaux.lib cxcore.lib highgui.lib PGRFlyCapture.lib FlexMS32.lib "
				OutputFile="RelWithDebInfo\camshift.exe"
				Version="0.0"
				GenerateManifest="TRUE"
				LinkIncremental="2"
				AdditionalLibraryDirectories="c:\yarp2\lib\$(OutDir)\$(OutDir),c:\yarp2\lib\$(OutDir),c:\yarp2\lib\$(OutDir)\Debug\$(OutDir),c:\yarp2\lib\$(OutDir)\Debug,c:\yarp2\lib\$(OutDir)\Release\$(OutDir),c:\yarp2\lib\$(OutDir)\Release,C:\ACE-5.6\ACE_wrappers\lib\$(OutDir),C:\ACE-5.6\ACE_wrappers\lib,&quot;C:\Program Files\OpenCV\lib\$(OutDir)&quot;,&quot;C:\Program Files\OpenCV\lib&quot;,C:\iCub\src\modules\dragonflyApi\winnt\lib\$(OutDir),C:\iCub\src\modules\dragonflyApi\winnt\lib,C:\vislab\dev\niflex\winnt\dd_orig\lib\$(OutDir),C:\vislab\dev\niflex\winnt\dd_orig\lib"
				ProgramDataBaseFile="$(OutDir)\camshift.pdb"
				GenerateDebugInformation="TRUE"
				SubSystem="1"
				StackReserveSize="10000000"/>
		</Configuration>
	</Configurations>
	<Files>
			<File
				RelativePath="C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt">
				<FileConfiguration
					Name="Debug|Win32">
					<Tool
					Name="VCCustomBuildTool"
					Description="Building Custom Rule C:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift/CMakeLists.txt"
					CommandLine="&quot;C:\Program Files\CMake 2.4\bin\cmake.exe&quot; &quot;-HC:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift&quot; &quot;-BC:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift&quot;"
					AdditionalDependencies="&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeSystem.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCCompiler.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXCompiler.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeSystemSpecificInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeGenericSystem.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\gcc.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\WindowsPaths.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows-cl.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeRCCompiler.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeRCInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCommonLanguageInclude.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCXXInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows-cl.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXPlatform.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCommonLanguageInclude.cmake&quot;;C:\yarp2\YARPConfig.cmake;C:\yarp2\conf\YarpDevice.cmake;C:\yarp2\yarpmodConfig.cmake;C:\yarp2\yarpmodDependencies.cmake;C:\yarp2\conf\FindOpenCV.cmake;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Templates\CMakeWindowsSystemConfig.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt&quot;;"
					Outputs="camshift.vcproj.cmake"/>
				</FileConfiguration>
				<FileConfiguration
					Name="Release|Win32">
					<Tool
					Name="VCCustomBuildTool"
					Description="Building Custom Rule C:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift/CMakeLists.txt"
					CommandLine="&quot;C:\Program Files\CMake 2.4\bin\cmake.exe&quot; &quot;-HC:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift&quot; &quot;-BC:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift&quot;"
					AdditionalDependencies="&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeSystem.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCCompiler.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXCompiler.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeSystemSpecificInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeGenericSystem.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\gcc.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\WindowsPaths.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows-cl.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeRCCompiler.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeRCInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCommonLanguageInclude.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCXXInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows-cl.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXPlatform.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCommonLanguageInclude.cmake&quot;;C:\yarp2\YARPConfig.cmake;C:\yarp2\conf\YarpDevice.cmake;C:\yarp2\yarpmodConfig.cmake;C:\yarp2\yarpmodDependencies.cmake;C:\yarp2\conf\FindOpenCV.cmake;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Templates\CMakeWindowsSystemConfig.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt&quot;;"
					Outputs="camshift.vcproj.cmake"/>
				</FileConfiguration>
				<FileConfiguration
					Name="MinSizeRel|Win32">
					<Tool
					Name="VCCustomBuildTool"
					Description="Building Custom Rule C:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift/CMakeLists.txt"
					CommandLine="&quot;C:\Program Files\CMake 2.4\bin\cmake.exe&quot; &quot;-HC:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift&quot; &quot;-BC:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift&quot;"
					AdditionalDependencies="&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeSystem.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCCompiler.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXCompiler.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeSystemSpecificInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeGenericSystem.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\gcc.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\WindowsPaths.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows-cl.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeRCCompiler.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeRCInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCommonLanguageInclude.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCXXInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows-cl.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXPlatform.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCommonLanguageInclude.cmake&quot;;C:\yarp2\YARPConfig.cmake;C:\yarp2\conf\YarpDevice.cmake;C:\yarp2\yarpmodConfig.cmake;C:\yarp2\yarpmodDependencies.cmake;C:\yarp2\conf\FindOpenCV.cmake;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Templates\CMakeWindowsSystemConfig.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt&quot;;"
					Outputs="camshift.vcproj.cmake"/>
				</FileConfiguration>
				<FileConfiguration
					Name="RelWithDebInfo|Win32">
					<Tool
					Name="VCCustomBuildTool"
					Description="Building Custom Rule C:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift/CMakeLists.txt"
					CommandLine="&quot;C:\Program Files\CMake 2.4\bin\cmake.exe&quot; &quot;-HC:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift&quot; &quot;-BC:/Documents and Settings/Vislab/Desktop/gsaponaro/camshift&quot;"
					AdditionalDependencies="&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeSystem.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCCompiler.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXCompiler.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeSystemSpecificInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeGenericSystem.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\gcc.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\WindowsPaths.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows-cl.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeRCCompiler.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeRCInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCommonLanguageInclude.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCXXInformation.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\Platform\Windows-cl.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCPlatform.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeFiles\CMakeCXXPlatform.cmake&quot;;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Modules\CMakeCommonLanguageInclude.cmake&quot;;C:\yarp2\YARPConfig.cmake;C:\yarp2\conf\YarpDevice.cmake;C:\yarp2\yarpmodConfig.cmake;C:\yarp2\yarpmodDependencies.cmake;C:\yarp2\conf\FindOpenCV.cmake;&quot;C:\Program Files\CMake 2.4\share\cmake-2.4\Templates\CMakeWindowsSystemConfig.cmake&quot;;&quot;C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\CMakeLists.txt&quot;;"
					Outputs="camshift.vcproj.cmake"/>
				</FileConfiguration>
			</File>
		<Filter
			Name="Source Files"
			Filter="">
			<File
				RelativePath="C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\camshift.cpp">
			</File>
			<File
				RelativePath="C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\YarpTimer.cpp">
			</File>
		</Filter>
		<Filter
			Name="Header Files"
			Filter="">
			<File
				RelativePath="C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\camshift.h">
			</File>
			<File
				RelativePath="C:\Documents and Settings\Vislab\Desktop\gsaponaro\camshift\YarpTimer.h">
			</File>
		</Filter>
	</Files>
	<Globals>
	</Globals>
</VisualStudioProject>
