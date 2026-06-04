import os
import threading
import rclpy
from rclpy.node import Node
from PIL import Image, ImageTk
import tkinter as tk
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import  Float32MultiArray ,UInt8MultiArray, UInt8
package_path = get_package_share_directory('r1_pkg_py')


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.sensor_sub =self.create_subscription( Float32MultiArray ,"/xyt_gui" ,self.readsensor,10)
        self.qr_sub =self.create_subscription(UInt8, "/Qr", self.qr_callback, 10)
        self.btn_pub = self.create_publisher(
                    UInt8MultiArray,
                    "/button_gui",
                    10
                )
        self.btn_timer =self.create_timer(0.05,self.timer1)
        self.bits = [0]*7
        self.sensor_data=[0]*10
        self.qr_cmd =0
        self.value =0
        self.buttons = []
        self.imu_data=[]*4

        self.name_btn =[
            "Start",
            "Auto_mode",
            "Manual_mode",
            "Debug_mode",
            "Switch_side",
            "Retry_Zone1",
            "Retry_Zone3",
        ]
        self.pictures = [
            "0.png",
            "1.png",
            "2.png",
            "3.png"
        ]
        self.gui_ready = False
        threading.Thread(target=self.start_gui, daemon=True).start()
    def readsensor(self, msg: Float32MultiArray):
        for i in range(min(10, len(msg.data))):
            self.sensor_data[i] = msg.data[i]
    # def imu_cmd(self, msg: Float32MultiArray):
    #         self.imu_data[0] = msg.data[0] #robot_vx
    #         self.imu_data[1] = msg.data[1]#robot_vy
    #         self.imu_data[2] = msg.data[2]#theta
    #         self.imu_data[3] = msg.data[3]#laser
     
    def qr_callback(self, msg:UInt8):
        self.qr_cmd = msg.data

    def timer1(self):
        msg = UInt8MultiArray()
        msg.data = self.bits.copy()

        self.btn_pub.publish(msg)
    
    def on_close(self):
        if self.root.winfo_exists():
            self.root.destroy()
        rclpy.shutdown()

    def start_gui(self):
        self.root = tk.Tk()
        self.root.geometry('1024x800')
        self.root.title('R1 GUI')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        

        self.left_frame = tk.Frame(self.root, bg="#1E3247", width=300)
        self.left_frame.pack(side="left", fill="both")
       

        #middle_size
        self.middle_frame = tk.Frame(self.root, bg="black", width=500, height=500)
        self.middle_frame.pack(side="left", fill="both", expand=True)

        #right_size
        self.right_frame = tk.Frame(self.root, bg="#265F98", width=300)
        self.right_frame.pack(side="left", fill="both")
        

        #label_left_button
        self.btn_left = tk.Label(self.left_frame, text="buttons", bg="lightgray" ,  font=("Arial", 14, "bold"))
        self.btn_left.grid(row=0, column=0, columnspan=3, pady=5)
        for r in range(7):
            btn = tk.Button(self.left_frame, text=self.name_btn[r],
                            width=10, height=3, font=("Arial", 12, "bold"),
                            bg="#0DA12D", fg="white", 
                            # activebackground="#1D4ED8",
                            relief="flat", cursor="hand2",
                            command=lambda idx=r: self.toggle_bit(idx))  # Direct command binding
            btn.grid(row=r+1, column=0, pady=1, padx=1)
            self.buttons.append(btn)
        
        self.image_label = tk.Label(self.middle_frame, bg="black")
        self.image_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.photo = None
        #label_middle
        self.mid = tk.Label(
            self.middle_frame,
            text="Display AprilTag",
            font=("Arial", 14, "bold"),
            fg='white',
            bg="#2B61B3"
        )

        self.mid.pack(pady=10)
        self.display_mode=tk.Label(self.right_frame, text="but", bg="#D1D5DB", font=("Arial", 14, "bold"))      
          #label_right
        self.sen_label = tk.Label(self.right_frame, text="Sensor panel", bg="#D1D5DB", font=("Arial", 14, "bold"))
        self.sen_label.grid(row=0, column=0, columnspan=3, pady=5)

        self.s1=self.create_label(self.right_frame,"x:0.0",1 ,0)
        self.s2=self.create_label(self.right_frame,"y:0.0",2 ,0)
        self.s3=self.create_label(self.right_frame,"rf_x:0.0",3 ,0)
        self.s4=self.create_label(self.right_frame,"rf_y:0.0",4 ,0)
        self.s5=self.create_label(self.right_frame,"theta:0.0",5 ,0)
        self.s6=self.create_label(self.right_frame,"ir_1:0",6 ,0)
        self.s7=self.create_label(self.right_frame,"ir_2:0",7 ,0)
        self.s8=self.create_label(self.right_frame,"ir_3:0",8 ,0)
        self.s9=self.create_label(self.right_frame,"ir_4:0",9 ,0)
        self.s10=self.create_label(self.right_frame,"ir_5:0",10 ,0)

        # self.s11=self.create_label(self.right_frame,"x:0.0",1 ,1)
        # self.s12=self.create_label(self.right_frame,"y:0.0",2 ,1)
        # self.s13=self.create_label(self.right_frame,"rf_x:0.0",3 ,1)
        # self.s14=self.create_label(self.right_frame,"rf_y:0.0",4 ,1)
        # self.s15=self.create_label(self.right_frame,"theta:0.0",5 ,1)
        # self.s16=self.create_label(self.right_frame,"ir_1:0",6 ,)
        # self.s17=self.create_label(self.right_frame,"ir_2:0",7 ,0)
        # self.s18=self.create_label(self.right_frame,"ir_3:0",8 ,0)
        # self.s19=self.create_label(self.right_frame,"ir_4:0",9 ,0)
        # self.s20=self.create_label(self.right_frame,"ir_5:0",10 ,0)

        self.gui_ready = True
        self.root.after(100, self.update_gui)
        self.root.mainloop()
    def updte_value(self):
        self.binary = ''.join(str(b) for b in reversed(self.bits))
        #print("bits", self.bits)
    def toggle_bit(self, bit_index):
        # Toggle between 0 and 1
        self.bits[bit_index] = 1 - self.bits[bit_index]
        self.updte_value()
        # Update button appearance to reflect ON/OFF state
        btn = self.buttons[bit_index]
        if self.bits[bit_index] == 1:
            btn.config(bg="#1D4ED8", activebackground="#0DA12D")  # ON state (blue)
        else:
            btn.config(bg="#0DA12D", activebackground="#1D4ED8")  # OFF state (green)
    def create_label(self,parent, text, row, col, **kwargs):
        lbl = tk.Button(parent, text=text, font=("Arial", 14, "bold"),
                    bg="#27496D",fg="#D1D5DB" ,**kwargs)
        lbl.grid(row=row, column=col, padx=5, pady=5, sticky='w')
        return lbl
    def update_gui(self):
        if self.gui_ready:
            self.s1.config(text=f"x:{self.sensor_data[0]:.3f}")
            self.s2.config(text=f"y:{self.sensor_data[1]:.3f}")
            self.s3.config(text=f"rf_x:{self.sensor_data[3]:.3f}")
            self.s4.config(text=f"rf_y:{self.sensor_data[4]:.3f}")
            self.s5.config(text=f"theta:{self.sensor_data[2]:.3f}")
            self.s6.config(text=f"ir_1:{self.sensor_data[5]}")
            self.s7.config(text=f"ir_2:{self.sensor_data[6]}")
            self.s8.config(text=f"ir_3:{self.sensor_data[7]}")
            self.s9.config(text=f"ir_4:{self.sensor_data[8]}")
            self.s10.config(text=f"ir_5:{self.sensor_data[9]}")
            self.mid.config(text=f"{self.pictures[self.qr_cmd]}")

            try:
                # SAFETY CHECK
                if self.qr_cmd < 0 or self.qr_cmd >= len(self.pictures):
                    return

                img_path = os.path.join(
                    package_path,
                    "apriltag",
                    self.pictures[self.qr_cmd]
                )
                if os.path.exists(img_path):

                    img = Image.open(img_path)
                    img = img.resize((600, 600), Image.LANCZOS)

                    new_photo = ImageTk.PhotoImage(img)

                    # FORCE UPDATE (IMPORTANT)
                    self.image_label.config(image=new_photo)
                    self.image_label.image = new_photo

                    self.photo = new_photo

                else:
                    self.image_label.config(
                        image="",
                        text="Image not found",
                        fg="white",
                        bg="black"
                    )

            except Exception as e:
                print("Image error:", e)
        self.root.after(100, self.update_gui)
        
def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()