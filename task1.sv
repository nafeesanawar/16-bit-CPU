module task1(input clk, input rst_n, input [7:0] start_pc, output[15:0] out);



  logic load_pc;
  logic next_pc;
  logic [7:0] program_counter_output;
  logic load_addr;
  logic data_address_register_input;
  logic data_address_register_output;


  logic ram_w_en;
  logic [7:0] ram_r_addr;
  logic ram_w_addr;
  logic ram_w_data;
  logic [15:0] ram_r_data;
  logic [7:0] ram_addr;

//controller output lab 7
  logic load_ir;
  logic sel_addr;
//  logic load_pc;
//  logic clear_pc;
  //logic load_addr;


  logic start;
  logic [2:0] opcode;
  logic [1:0] ALU_op;
  logic [1:0] shift_op;
  logic Z;
  logic N;
  logic V;
  logic waiting;
  logic [1:0] reg_sel;
  logic [1:0] wb_sel;
  logic w_en;
  logic en_A;
  logic en_B;
  logic en_C;
  logic en_status;
  logic sel_A;
  logic sel_B;

  logic [15:0] ir;
 logic [1:0] reg_sel;
//  logic [2:0] opcode;
//  logic [1:0] ALU_op;
//  logic [1:0] shift_op;
  logic [15:0] sximm5;
  logic [15:0] sximm8;
  logic [2:0] r_addr;
  logic [2:0] w_addr;


  logic [15:0] cpu_module;
  logic load;
  logic [15:0] ins_reg_output;



  logic [15:0] mdata;
  logic [7:0] pc;
  /*
  logic [1:0] wb_sel;
  logic [2:0] w_addr;
  logic w_en;
  logic [2:0] r_addr;
  logic en_A;
  logic en_B;
  logic [1:0] shift_op;
  logic sel_A;
  logic sel_B;
  logic [1:0] ALU_op;
  logic en_C;
  logic en_status;
	logic [15:0] sximm8;
  logic [15:0] sximm5;
  logic [15:0] datapath_out;
  logic Z_out;
  logic N_out;
  logic V_out;
  */



  controller controller(.clk(clk), .rst_n(rst_n),.start(start),.opcode(opcode), .ALU_op(ALU_op), .shift_op(shift_op),
  .Z(Z),.N(N), .V(V),.waiting(waiting), .reg_sel(reg_sel), .wb_sel(wb_sel), .w_en(w_en), .en_A(en_A),.en_B(en_B),
  .en_C(en_C), .en_status(en_status),.sel_A(sel_A), .sel_B(sel_B),.load_ir(load_ir), .ram_w_en(ram_w_en),.sel_addr(sel_addr),
  .load_pc(load_pc), .clear_pc(clear_pc), .load_addr(load_addr));


  idecoder idecoder(.ir(ins_reg_output), .reg_sel(reg_sel), .opcode(opcode), .ALU_op(ALU_op), .shift_op(shift_op), .sximm5(sximm5),
    .sximm8(sximm8), .r_addr(r_addr), .w_addr(w_addr));


//not sure if cpu module connected with ram_r_data is correct
ins_reg ins_reg (.clk(clk), .cpu_module(ram_r_data), .load(load), .ins_reg_output(ins_reg_output));


program_counter program_counter (.clk(clk), .load_pc(load_pc), .load(load), .next_pc(next_pc), .program_counter_logic(program_counter_output));


data_address_register data_address_register (.clk(clk), .load_addr(load_addr), .data_address_register_input(data_address_register_input),
.data_address_register_output(data_address_register_output));

ram ram (.clk(clk), .ram_w_en(ram_w_en),.ram_r_addr(ram_addr),.ram_w_addr(ram_addr), .ram_w_data(data_address_register_input), .ram_r_data(ram_r_data));


datapath datapath(.clk(clk),.mdata(mdata), .pc(pc),.wb_sel(wb_sel),.w_addr(w_addr),.w_en(w_en),.r_addr(r_addr),
.en_A(en_A),.en_B(en_B),.shift_op(shift_op),.sel_A(sel_A),.sel_B(sel_B),.ALU_op(ALU_op),
.en_C(en_C),.en_status(en_status),.sximm8(sximm8),.sximm5(sximm5),.datapath_out(datapath_out),
.Z_out(Z),.N_out(N),.V_out(V));


assign next_pc = (clear_pc) ? start_pc : program_counter_output+1;
assign ram_addr = (sel_addr) ? program_counter_output : data_address_register_output;



endmodule: task1




module program_counter (input clk, input load_pc, input [7:0] next_pc, output reg [7:0] program_counter_output);
  wire [15:0] program_counter_output_change_wire;
  assign program_counter_output = (load_pc) ? next_pc : program_counter_output;
  always @(posedge clk)
    program_counter_output = program_counter_output_change_wire;
endmodule: program_counter


module data_address_register(input clk, input load_addr, input [6:0] data_address_register_input, output reg [7:0] data_address_register_output);
  wire [15:0] data_address_register_output_change_wire;
  assign data_address_register_output = (load_addr) ? data_address_register_input : data_address_register_output;
  always @(posedge clk)
    data_address_register_output = data_address_register_output_change_wire;
endmodule: data_address_register



// op is not given as an input to the fsm although it is in the fsm input -> why is that
// need to add default statement
// not sure if every state is doing what it is supposed to do

module controller(
  input clk,
  input rst_n,
  input start,
  input [2:0] opcode,
  input [1:0] ALU_op,
  input [1:0] shift_op,
  input Z,
  input N,
  input V,
  output waiting,
  output [1:0] reg_sel,
  output [1:0] wb_sel,
  output w_en,
  output en_A,
  output en_B,
  output en_C,
  output en_status,
  output sel_A,
  output sel_B,

//lab 7
    output load_ir,
    output ram_w_en,
    output sel_addr,
    output load_pc,
    output clear_pc,
    output load_addr,
    output control_signal);


                logic sel_A_a;
                assign sel_A = sel_A_a;


                logic sel_B_a;
                assign sel_B = sel_B_a;

                logic en_status_a;
                assign en_status = en_status_a;

                logic en_C_a;
                assign en_C = en_C_a;

                logic en_B_a;
                assign en_B = en_B_a;

                logic en_A_a;
                assign en_A = en_A_a;

                logic [1:0] reg_sel_a;
                assign reg_sel = reg_sel_a;

                logic waiting_a;
                assign waiting = waiting_a;


                logic w_en_a;
                assign w_en = w_en_a;

                logic [1:0] wb_sel_a;
                assign wb_sel = wb_sel_a;

                  logic [4:0] state;
                  parameter waiting_state = 1;
                  parameter decode_state = 2;
                  parameter move_state = 3;
                  parameter getA = 4;
                  parameter getB = 5;
                  parameter ALU_state = 6;
                  parameter write_register_state = 7;
                  parameter mov_instruction_one = 8;
                  parameter mov_instruction_two = 9;


                  always_ff @(posedge clk)
                  if(~rst_n)
                  state <= waiting_state;
                  else

                  case(state)

                  waiting_state:
                  begin
                  if(start === 1)
                    state <= decode_state;
                  else
                  state <= waiting_state;
                  end

                  decode_state:
                  begin
                  if (opcode == 3'b110)
                    state <= move_state;
                  if (opcode == 3'b101)
                  state <= getA;
                  end

                  getA:
                  begin
                  state <= getB;
                  end

                  getB:
                  begin
                  state <= ALU_state;
                  end

                  ALU_state:
                  begin
                  state <= write_register_state;
                  end

                  write_register_state:
                  begin
                  state <= waiting_state;
                  end


                  move_state:
                  begin
                  if (ALU_op == 2'b10)
                    state <= mov_instruction_one;
                  if (ALU_op == 2'b00)
                    state <= mov_instruction_two;
                  end

                  mov_instruction_one:
                  begin
                    state <= waiting_state;
                  end

                  mov_instruction_two:
                  begin
		    state <= write_register_state;
                  end


                  default: state <= waiting_state;

                  endcase

                  always_comb begin
                    case(state)

                    waiting_state:
                    begin

		    wb_sel_a = 2'b00;
                    sel_A_a = 1'b1;
                    sel_B_a = 1'b1;
                    waiting_a = 1;
                    w_en_a = 0;
                    en_A_a = 0;
                    en_B_a = 0;
                    en_C_a = 0;
                    en_status_a = 0;
		    reg_sel_a = 2'b11;
                    end

                    decode_state:
                    begin
                    waiting_a = 0;
		    wb_sel_a = 2'b00;
		    w_en_a =0;
 		    en_C_a = 0;
		    en_status_a = 0;
                    en_B_a = 0;
                    sel_A_a = 1;
                    sel_B_a = 1;
                    reg_sel_a = 2'b11;
                    en_A_a = 0;
                    end



                    mov_instruction_one:
                    begin

		    waiting_a = 0;
 		    en_C_a = 0;
		    en_status_a = 0;
                    en_B_a = 0;
                    sel_A_a = 1;
                    sel_B_a = 1;
                    en_A_a = 0;
                    reg_sel_a = 2'b10;
                    w_en_a = 1;
                    wb_sel_a = 2'b10;
                    end


                    mov_instruction_two:
                    begin


		    waiting_a = 0;
		    wb_sel_a = 2'b00;
                    sel_B_a = 1;
                    en_A_a = 0;
                    reg_sel_a = 2'b00;
                    w_en_a = 0;
                    en_B_a = 1;
                    sel_A_a = 1;
                    en_C_a = 1;
                    en_status_a = 1;
                    end

                    getA:
                    begin

		    waiting_a = 0;
		    wb_sel_a = 2'b00;
		    w_en_a = 0;
 		    en_C_a = 0;
		    en_status_a = 0;
                    en_B_a = 0;
                    sel_A_a = 0;
                    sel_B_a = 0;
                    reg_sel_a = 2'b10;
                    en_A_a = 1;
                    end

                    getB:
                    begin

		    waiting_a = 0;
		    wb_sel_a = 2'b00;
		    w_en_a = 0;
 		    en_C_a = 0;
		    en_status_a = 0;
                    reg_sel_a = 2'b00;
                    en_A_a = 0;
                    en_B_a = 1;
                    sel_A_a = 0;
                    sel_B_a = 0;
                    end

                    ALU_state:
                    begin

		    waiting_a = 0;
		    reg_sel_a = 2'b11;
		    wb_sel_a = 2'b00;
		    w_en_a = 0;
                    en_A_a = 0;
		    en_B_a = 0;
 		    en_C_a = 1;
		    en_status_a = 1;
		    sel_A_a = 0;
		    sel_B_a = 0;

                    end


                    write_register_state:
                    begin
		    waiting_a = 0;
		    reg_sel_a = 2'b01;
		    wb_sel_a = 2'b00;
		    w_en_a = 1;
                    en_A_a = 0;
		    en_B_a = 0;
 		    en_C_a = 0;
		    en_status_a = 0;
		    sel_A_a = 0;
		    sel_B_a = 0;

                    end

		    default: begin
		    waiting_a = 0;
		    reg_sel_a = 2'b11;
		    wb_sel_a = 2'b00;
		    w_en_a = 0;
                    en_A_a = 0;
		    en_B_a = 0;
 		    en_C_a = 0;
		    en_status_a = 0;
		    sel_A_a = 1;
		    sel_B_a = 1;
		    end

                    endcase
                  end

endmodule: controller




module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output [2:0] opcode, output [1:0] ALU_op, output [1:0] shift_op,
		output [15:0] sximm5, output [15:0] sximm8,
                output [2:0] r_addr, output [2:0] w_addr);



    // assign opcode t0 [15:13] of ir
    assign opcode = ir [15:13];
    // assign opcode t0 [12:11] of ir
    assign ALU_op =  ir [12:11];
    // assign opcode t0 [4:0] of ir
    wire [4:0] imm5;
    // assign ir value to imm5
    assign imm5 = ir [4:0];
    // take the imm5 wire and sign extend: taking the MSB (4 here) and copy that to the rest of the wires
    assign sximm5 = {{11{imm5[4]}}, imm5};
    // imm5 is 5 wires -> 5 bits.
    // creat 9 4s imm5 and adding them to the prexisting imm5
    // imm8 -> same as imm5
    wire [7:0] imm8;
    assign imm8 = ir [7:0];
    assign sximm8 = {{8{imm8[7]}}, imm8};
    //0123456789_10_11_12_13_14_15 -> 0123456777777777
    assign shift_op = ir [4:3] ;


    // mux wires

    wire [10:8] Rn;
    wire [7:5] Rd;
    wire [2:0] Rm;

    logic [2:0] mux_out; // combine r_addr and w_addr

    always_comb begin

      case(reg_sel)
      2'b00: begin
      mux_out = ir [2:0] ;
      end

      2'b01: begin
      mux_out = ir [7:5] ;
      end

      2'b10: begin
      mux_out = ir [10:8] ;
      end

      default: begin
      mux_out = 3'b000;
      end
      endcase

    end
      assign r_addr = mux_out;
      assign w_addr = mux_out;

endmodule: idecoder



module ins_reg (input clk, input [15:0] cpu_module, input load, output reg [15:0] ins_reg_output);
  wire [15:0] ins_reg_output_change_wire;
  assign ins_reg_output_change_wire = (load) ? cpu_module : ins_reg_output;
  always @(posedge clk)
    ins_reg_output = ins_reg_output_change_wire;
endmodule: ins_reg






module datapath(
  input clk,
  input [15:0] mdata,
  input [7:0] pc,
  input [1:0] wb_sel,
  input [2:0] w_addr,
  input w_en,
  input [2:0] r_addr,
  input en_A,
  input en_B,
  input [1:0] shift_op,
  input sel_A,
  input sel_B,
  input [1:0] ALU_op,
  input en_C,
  input en_status,
	input [15:0] sximm8,
  input [15:0] sximm5,
  output [15:0] datapath_out,
  output Z_out,
  output N_out,
  output V_out);










wire [15:0] pc_connect;
assign pc_connect = {8'b0, pc};
wire [15:0] w_data;
    //mux #9 --> illegal reference to net w_data but why?
front_mux front_mux(.C(datapath_out),.pc(pc_connect),.sximm8(sximm8),.mdata(mdata),.wb_sel(wb_sel), .w_data(w_data));
//what is datapath_in in the new lab???
//is w_data be connected with w_data?





  //6
  //internal wires
  wire [15:0] three_out; //outut of register A
  wire [15:0] val_A; // output of mux or number 6
  // mux # 6
  assign val_A = sel_A ?  16'b0: three_out; //if sel_A = 1, choose 16'b0. If 0, choose three_out



  //7
  //internal wires
  wire [15:0] shift_out;
  wire [15:0] val_B;
  //mux #7
  assign val_B = sel_B ? sximm5 : shift_out; //if sel_B = 1, choose sximm5. if 0, choose shift_out



  // ALU instantiation -> #2
  wire [2:0] output_wire;
  wire [2:0] output_status;
  wire [15:0] ALU_out;
  ALU ALU(.val_A(val_A),
  .val_B(val_B),
  .ALU_op(ALU_op),
  .output_status(output_wire),
  .ALU_out(ALU_out));

  assign Z_out = output_status[2];
  assign N_out = output_status[1];
  assign V_out = output_status[0];


  //shift_in is connected with the output of reg B. Thus we need to create an internal wire
  wire [15:0] four_out;
  // shifter instantiation -> #8
  shifter shifter(.shift_in(four_out),
  .shift_out(shift_out),
  .shift_op(shift_op));




  // regfile
  // internal wire that go into regfile
  //wire [15:0] w_data
  wire [15:0] r_data;
  regfile regfile(.w_data(w_data),
  .w_addr(w_addr),
  .w_en(w_en),
  .r_addr(r_addr),
  .r_data(r_data),
  .clk(clk));



  register A(.register_input_one(r_data),
  .register_input_two(en_A),
  .clk(clk),
   .register_output(three_out));

  register B(.register_input_one(r_data),
   .register_input_two(en_B),
   .clk(clk),
   .register_output(four_out));


    register C(.register_input_one(ALU_out),
    .register_input_two(en_C),
    .clk(clk),
    .register_output(datapath_out));

    status status(.Z(output_wire),
    .en_status(en_status),
    .clk(clk),
    .output_status(output_status));


endmodule: datapath




module register(input [15:0] register_input_one, input clk, input register_input_two, output reg [15:0] register_output);
    always @(posedge clk)begin
    if(register_input_two == 1'b1)begin
      register_output <=  register_input_one;
    end
    end
endmodule: register



module status(input [2:0] Z, input clk, input en_status, output reg [2:0] output_status);
    always @(posedge clk)begin
    if(en_status == 1'b1)begin
      output_status <= Z;
    end
    end
endmodule: status


module front_mux(input [15:0] mdata,
  input [15:0] sximm8,
   input [15:0] pc,
   input [15:0] C,
   input [1:0] wb_sel,
   output reg [15:0] w_data);
    always_comb begin
        case (wb_sel)
            2'b11: w_data = mdata;
            2'b10: w_data = sximm8;
            2'b01: w_data = {{8{1'b0}},pc};
            2'b00: w_data = C;
        endcase
    end
endmodule:front_mux



module ALU(
  input [15:0] val_A,
   input [15:0] val_B,
   input [1:0] ALU_op,
   output [15:0] ALU_out,
   output [2:0] output_status);

   reg [15:0] output_ALU_out;
   assign ALU_out = output_ALU_out;
   reg [2:0] status;
   assign output_status = status;

   //the ALU produces a single-bit status output that is true if and only if ALU_out is equal to zero.
  always_comb begin
//ALU conditions specified by lab
      case (ALU_op)
          2'b00:
          output_ALU_out = val_A+val_B;

          2'b01:
          output_ALU_out = val_A-val_B;

          2'b10:
          output_ALU_out = val_A&val_B;

          2'b11:
          output_ALU_out = ~val_B;

      endcase
    end

    always @(*)begin
	casex (output_ALU_out)
  //Z_out
            16'b0000000000000000: status [2:0] = 3'b100;
    //N_out
            16'b1xxxxxxxxxxxxxxx: status [2:0] = 3'b010;
    // V_out
            16'b0xxxxxxxxxxxxxxx: status [2:0] = 3'b001;
            default: status [2:0] = 3'b000;
        endcase
        //this shows overflow
        status[0] = val_A ^ val_B;
	end



endmodule: ALU



module shifter(
  input [15:0] shift_in,
   input [1:0] shift_op,
   output reg [15:0] shift_out);


logic [15:0] last_logic;


always @(*) begin
  case(shift_op)
    2'b00:
      shift_out = shift_in;

    2'b01:
      shift_out = shift_in << 1;

    2'b10:
      shift_out = shift_in >> 1;

    2'b11: begin
      last_logic = shift_in >>> 1;
      shift_out = {shift_in[15],last_logic[14:0]};
    end

    default:
      shift_out = 16'b0;
  endcase

end
endmodule: shifter
