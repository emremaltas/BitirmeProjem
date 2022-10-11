# Da Vinci Robot Optimizasyonu Projesi


- ***Biyomedikal Mühendisliği Bitirme Projesi*** dersi kapsamında hazırlanmıştır.
- Projenin fikri, tüm kodların yazılması ve projenin devre haline getirilmesi işlemleri tarafımca 3 ayda yapılmıştır.
- Proje kapsamında yazılmış  ***Bitirme Projesi Tezi***'ne erişmek için aşağıdaki bağlantıya tıklayınız.

## Projenin Şematiği
![Bitirme Tezi Şematiği](https://github.com/emremaltas/BitirmeProjem/blob/master/tez%20şema.png)

## Özet

- Proje kapsamında Da Vinci Robotik Cerrahi Sistemini minimize ederek, sistemin sahip olduğu özelliklere yeni özellikler eklemeye çalıştım.
- Da Vinci Robot Optimizasyonu projesi kapsamında, ***yüz tespiti***, ***göz bebeği tespiti***, ***renk tespiti***, ***tespit edilmek istenen rengin parametrelerinin ayarlandığı menü***, ***tespit edilen renkli cismin alanı ve merkez noktası tespiti*** işlemi yapılmış ve ön güvenlik için şifre giriş devresive sistemde kullanılan 6 eksen robot kol kontrolü için devre oluşturulmuştur.  
- Projede 1 adet 6 eksen robot kol kullanılmış ve robot koldan ayrı olarak üstünde konumlandırılan kameradan görüntüler bilgisayar ortamına alınmıştır.
- Kameradan gelen görüntüler üzerinde renk tespiti gerçekleştirilebilir olup bu tespiti takiben alan ve koordinat tespiti işlemleri yapılmış ve bilgisayar ekranında operatöre sunulmuştur. 
- Eş zamanlı olarak bilgisayar kamerasından görüntüler alınmış, üzerinde yüz ve göz bebeği tespiti işlemi gerçekleştirilmiştir.
- Yüz ve gözbebeği tespiti robot kolun kontrolünün sağlanabilmesi için gerekli olup tespit edilememe durumu belirli bir süreyi aştığında ise robot kol kontrol devresi pasif hale getirilmiştir. 
- Projeye eklemiş olduğum yüz ve göz bebeği tespiti işlemi, kullanım aşamasında güvenliği sağlamak amaçlı olup sistemin kullanılabilirliğini de yetkin kişiler ile sınırlamak için sisteme şifreli giriş kontrol devresi eklenmiştir.
- Robot kol kontrolü için şifrenin doğru girilmesi ve yüz, göz bebeği tespitinin gerçekleşiyor olması gerekmektedir. 

- Görüntü işleme yazılımları ***C++ programlama dili*** ile ***OpenCv kütüphanesi*** kullanılarak ***Visual Studio*** ortamında yazılmıştır.

- Robot kontrol devresi yazılımları, ***C programlama dili*** ile ***STM32F407VG Discovery kartı***  ***Registerlara erişilerek***  ***Atollic TrueSTUDIO*** ortamında yazılmıştır. 

- Şifre kontrol devresi yazılımları ***C programlama dili*** ile ***PIC16F877/A mikrodenetleyicisi*** kullanılarak  ***PIC C Compiler*** ortamında yazılmıştır. 


## Proje kapsamında yazılmış tezin tam  haline ulaşmak için [tıklayınız.](https://github.com/emremaltas/BitirmeProjem/blob/master/Emre_MALTAŞ_181020037%20.pdf)
