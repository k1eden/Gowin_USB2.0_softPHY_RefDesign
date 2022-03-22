________________________________________________________________________
  Example USB 2.0 SoftPHY Design Read Me
-------------------------------------------------------------------------
Object device:GW1NSR-4-MBGA64P
---------------------------------------------------------------------------
File List:
---------------------------------------------------------------------------
.
|-- doc
|   `-- Readme.txt                              -->  Read Me file (this file)
|-- tb 
|   `-- tb.v                                    -->  TestBench for example design
|   `-- prim_sim.v                              -->  Gowin Simulation lib
|   `-- prim_sim_h.v                            -->  Gowin Simulation lib
|   |-- usb_host
|   |   `--usbh_crc5.v                          -->  File for USB Simulation
|   |   `--usbh_crc16.v                         -->  File for USB Simulation
|   |   `--usbh_fifo.v                          -->  File for USB Simulation
|   |   `--usbh_host.v                          -->  File for USB Simulation
|   |   `--usbh_host_defs.v                     -->  File for USB Simulation
|   |   `--usbh_sie.v                           -->  File for USB Simulation
|   |   `--usb_define.v                         -->  File for USB Simulation
|-- project
|   `-- usb_refdesign.gprj                      -->  Gowin Project File for Example Design
|   `-- usb_refdesign.gprj.user                 -->  Gowin Project File for Example Design
|   |-- impl
|   |   `-- project_process_config.json
|   |   |-- synthesize   
|   |   |-- temp                             
|   |-- src                          
|       `-- TOP.v                               -->  File for Gowin Project
|       `-- usb_descriptor.v                    -->  File for Gowin Project
|       `-- usb_ref.cst                         -->  File for Gowin Project
|       `-- usb_ref.sdc                         -->  File for Gowin Project
|       |-- gowin_pllvr
|       |   `-- gowin_pllvr.v                   -->  File for Gowin Project
|       |   `-- gowin_pllvr.mod                 -->  File for Gowin Project
|       |   `-- gowin_pllvr.ipc                 -->  File for Gowin Project
|       |   `-- gowin_pllvr_tmp.v               -->  File for Gowin Project
|       |   |-- temp
|       |-- fifo_sc_top
|       |   `-- fifo_sc_top.v                   -->  File for Gowin Project(Encrypted)
|       |   `-- fifo_sc_top.vo                  -->  File for Simulation
|       |   `-- fifo_sc_top.ipc                 -->  File for Gowin Project
|       |   `-- fifo_sc_top_tmp.v               -->  File for Gowin Project
|       |   |-- temp
|       |-- usb2_0_softphy
|       |   `-- usb2_0_softphy.v                   -->  File for Gowin Project(Encrypted)
|       |   `-- usb2_0_softphy.vo                  -->  File for Simulation
|       |   `-- usb2_0_softphy.ipc                 -->  File for Gowin Project
|       |   `-- usb2_0_softphy_tmp.v               -->  File for Gowin Project
|       |   |-- temp
|       |-- usb_device_controller
|       |   `-- usb_device_controller.v         -->  File for Gowin Project(Encrypted)
|       |   `-- usb_device_controller.vo        -->  File for Simulation
|       |   `-- usb_device_controller.ipc       -->  File for Gowin Project
|       |   `-- usb_device_controller_tmp.v     -->  File for Gowin Project
|       |   |-- temp
|-- simulation                                  -->  Simulation Environment
|       |--modelsim
|       |   `--tb.do
|       |   `--wave.do
|       |   `--run_sim.bat
|       |   `--readme.txt

---------------------------------------------------------------------------------------------------------------
HOW TO OPEN A PROJECT IN Gowin:
---------------------------------------------------------------------------------------------------------------
1. Unzip the respective design files.
2. Launch Gowin and select "File -> Open -> Project"
3. In the Open Project dialog, enter the Project location -- "project",select the project"usb_refdesign.gprj".
4. Click Finish. Now the project is successfully loaded. 

---------------------------------------------------------------------------------------------------------------
HOW TO RUN SYNTHESIZE, PLACE AND ROUTE, IP CORE GENERATION, AND TIMING ANALYSIS IN Gowin:
---------------------------------------------------------------------------------------------------------------

1. Click the Process tab in the process panel of the Gowin dashboard. 
   Double click on Synthsize. This will bring the design through synthesis.
2. Click the Process tab in the process panel of the Gowin dashboard. 
   Double click on Place & Route. This will bring the design through mapping, place and route.
3. Once Place & Route is done, user can double click on Timing Analysis Report to get 
   the timing analysis result.
4. Click on "Project -> Configuration -> Place & Route" to configurate the Post-Place File 
   and SDF File of the design.
----------------------------------------------------------------------------------------------------------------

HOW TO RUN SIMULATION
1. User can run functional simulation by software modelsim. 
----------------------------------------------------------------------------------------------------------------

HOW TO  GENERATE IP CORE
1. Click the IP Core Generator tab in the Window panel of the Gowin dashboard.
   Double click on "USB 2.0 SoftPHY". This will generate the IP Core for the design.
--------------------------------------------------------------------------------------------------------------

