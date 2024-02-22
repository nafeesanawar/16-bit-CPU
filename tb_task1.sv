module tb_task1(output err);

  reg error_line;
  assign err = error_line;

  reg clk;
  reg rst_n;
  reg [7:0] start_pc;
  wire[15:0] out;

  task1 dut(.clk(clk), .rst_n(rst_n), .start_pc(start_pc), .out(out));

  initial begin
  forever begin
  clk = 1;
  #10;
  clk = 0;
  #10;
  end
  end

initial begin
rst_n = 0;
#10;
rst_n = 1;
#10;
// starting the program count at 0. So we are going to take the instruction
//from zero address (automatically), fsm is going to change the program count mux and change to 0
// so the program count is going to be incremented
start_pc = 0;
#20;
//look at your dut and goes insed the task1 module and look for the program counter logic
//if start_pc wire is connected properly program counter logic would output 0
//all of this staffs are happening in wait state. So FSM is still in wait state
if (dut.program_counter_logic == 8'b0)begin
  $display("PASS: Program counter output good connection");
end else begin
  error_line = 1;
  $display("FAIL: Check program counter output connection");
end

if(dut.ram_addr == 8'b0)begin
  $display("PASS: Ram_addr is properly connected to the MUX");
end else begin
error_line = 1;
$display("FAIL: Ram_addr is not properly connected to the MUX");
end

//now we want to make sure sure ram_w_en is zero because that is going to allow us to read data from the ram
if(dut.ram_w_en == 8'b0)begin
  $display("PASS: Now you can read data/instruction from the RAM");
end else begin
error_line = 1;
$display("FAIL: WE can not read data/instruction from the RAM");
end

#20;  // checking if ram_r_data is connected properly and if ram has actually given us data AKA the first instruction after changing the text file with numbers
if(dut.ram_r_data == 16'b1101001000000101)begin
  $display("PASS: ram_r_data is connected properly");
end else begin
error_line = 1;
  $display("FAIL: ram_r_data is not connected properly");
end

//assuming the above tests pass, now we have the MOV instructio ready to go to the instruction register
#200; // this is just giving us time move through the states of fsm

// after 200 seconds all of our mov isntructions should be done and we should be in the wait state
// so now we need to monitor whether or not we are going to halt state

//assuming halt state is going to be state 10
if(dut.controller.state == 10)begin
  $display("PASS: goes to halt state from wait state ");
end else begin
  error_line = 1;
  $display("FAIL: Does not go to halt state from wait state ");
end


// once you are in halt state -> we stop fetching instruction. So in ram_w_en = 1, then the ram would be writing the data instead
// of reading it
if(dut.ram_w_en == 8'b1)begin
  $display("PASS: Instruction fetching stopped after Halt state ");
end else begin
error_line = 1;
$display("FAIL:  Instruction fetching didn't stop after Halt state  ");
end








$stop;
end
endmodule: tb_task1
