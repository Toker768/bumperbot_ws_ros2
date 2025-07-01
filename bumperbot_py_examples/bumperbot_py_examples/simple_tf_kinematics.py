import rclpy
from rclpy.node import Node


from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster  # İki koordinat sistemi (örneğin: base ve top) arasındaki sabit mesafeyi yaymak için kullanılır.
from geometry_msgs.msg import TransformStamped # Bu iki çerçeve (frame) arasında şu kadar mesafe ve şu kadar dönüklük var." bilgisini tutar. Zamanla birlikte.
from tf2_ros import TransformBroadcaster   


# | Kavram                       | Anlamı                                                        |
# | ---------------------------- | ------------------------------------------------------------- |
# | `TransformStamped`           | İki çerçeve arasındaki konum & yön bilgisi (zamanla birlikte) |
# | `StaticTransformBroadcaster` | Sabit ilişkileri yayınlayan sınıf                             |
# | `TransformBroadcaster`       | Zamanla değişen ilişkileri yayınlayan sınıf                   |
# | `sendTransform()`            | TF bilgisini ROS ağına yayınla                                |
# | `Node`                       | ROS içindeki aktif çalışan birim                              |
# | `timerCallback()`            | Zamanlayıcıyla periyodik çağırılan fonksiyon                  |



class SimpleTfKinematics(Node):

    def __init__(self):
        super().__init__("simple_tf_kinematics")

        # TF Broadcaster
        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        # ROS’a, “şu iki parçanın birbirine göre sabit pozisyonu var” mesajını gönderecek bir yayıncı oluşturduk.
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)


        self.static_transform_stamped_ = TransformStamped()  # “Kamerayı robota taktım, ama konumunu bir kâğıda yazmalıyım ki herkes bilsin. ” İşte bu kağıt = TransformStamped 
        self.dynamic_transform_stamped_ = TransformStamped()



        self.x_increment_ = 0.05 
        self.last_x_ = 0.0  # Robotun en son konumu 



        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg() # ROS, her şeyin zamanını takip eder. Burada “şu anki zaman” bilgisini ekledik.
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"  # (kimden) Bu transform hangi çerçeveye bağlı? → Ana çerçevemiz: bumperbot_base Yani robotun altı gibi düşünebiliriz.  -->  robotun gövdesi 
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"   # (kime) Bu çerçeve neye bağlanıyor? → bumperbot_top Bu da robotun üst kısmı gibi düşün. “Altın üstü” gibi.    --> robotun kafası
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3  # üst parça, alt parçanın 30 cm yukarısında
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0 # (0, 0, 0, 1) → Bu, hiç dönmemiş anlamına gelir.Yani üst parça tam düz, tabanla paralel şekilde duruyor.
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0


        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_) # “Hey ROS! ‘bumperbot_base’ ile ‘bumperbot_top’ arasında sabit bir transform var, yukarıya doğru 30cm, ve düz şekilde.”
        self.get_logger().info("Publishing static transform between %s and %s" % 
                      (
                        self.static_transform_stamped_.header.frame_id,
                        self.static_transform_stamped_.child_frame_id
                      ))
        
        # Timer
        self.timer_ = self.create_timer(0.1, self.timerCallback)

    def timerCallback(self):
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0
        self.dynamic_transform_stamped_.transform.rotation.x = 0.0
        self.dynamic_transform_stamped_.transform.rotation.y = 0.0
        self.dynamic_transform_stamped_.transform.rotation.z = 0.0
        self.dynamic_transform_stamped_.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x
     



def main():
    rclpy.init()

    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#     Adım adım:
# 📦 transform = TransformStamped()
# → "Bu parça şuraya bağlı, şu kadar uzakta" diye bir bilgi kutusu hazırla

# 🚚 broadcaster = StaticTransformBroadcaster(self)
# → "Ben bu kutuları ROS sistemine dağıtmak için hazırım" de

# 📬 broadcaster.sendTransform(transform)
# → "Al bu kutuyu, ROS'a bildir" komutunu ver

