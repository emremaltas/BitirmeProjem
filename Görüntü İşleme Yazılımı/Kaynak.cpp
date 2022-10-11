// BitirmeProjesi.cpp : Bu dosya 'main' iþlevi içeriyor. Program yürütme orada baþlayýp biter.
//4.04.2022 Emre  Maltas

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void islem_sec_ekrani();

VideoCapture IPcam;
VideoCapture PCcam;

Mat bilgisayar_kam_gri, bilgisayar_kam;
Mat ip_kam_gri, ip_kam;

int mavi1 = 0, yesil1 = 0, kirmizi1 = 0, mavi2 = 0, yesil2 = 0, kirmizi2 = 0;

int mavi1_gr = 0, yesil1_gr= 0, kirmizi1_gr = 0, mavi2_gr = 0, yesil2_gr = 0, kirmizi2_gr = 0;
int mavi1_gl = 0, yesil1_gl = 0, kirmizi1_gl = 0, mavi2_gl = 0, yesil2_gl = 0, kirmizi2_gl = 0;

string IP_Adresi = "http://192.168.1.102:8080/video", CaseCadeYuz = "haarcascade_frontalface_default.xml",
CaseCadeGoz = "haarcascade_lefteye_2splits.xml", CaseCadeGoz_sag = "haarcascade_righteye_2splits.xml ", t_penceresi = "TrackBar Penceresi";

uint8_t  sayac = 0;

void TrackBar_git()
{

	namedWindow(t_penceresi, WINDOW_AUTOSIZE);
	createTrackbar("Mavi1", t_penceresi, &mavi1, 255);
	createTrackbar("Yesil1", t_penceresi, &yesil1, 255);
	createTrackbar("Kirmizi1", t_penceresi, &kirmizi1, 255);
	createTrackbar("Mavi2", t_penceresi, &mavi2, 255);
	createTrackbar("Yesil2", t_penceresi, &kirmizi2, 255);
	createTrackbar("Kirmizi2", t_penceresi, &yesil2, 255);

}

void IP_Kamera_Ayarlari()
{
	IPcam.open(IP_Adresi);
	if (!IPcam.isOpened())
	{
		cout << "IP Kamerasi Acilmadi!!" << endl;
	}
	else if (IPcam.isOpened())
	{
		cout << "IP Kamerasi Acildi!!" << endl;
	}
}
void Bilgisayar_Kamerasý_Ayarlari()
{
	PCcam.open(0);
	if (!PCcam.isOpened())
	{
		cout << "Bilgisayar Kamerasi Acilmadi!!" << endl;
	}
	else if (PCcam.isOpened())
	{
		cout << "Bilgisayar Kamerasi Acildi!!" << endl;
	}
}


void renk_tespit_degeri_bul()
{
	IP_Kamera_Ayarlari();
	
	Mat orijinal, gri;
	
	TrackBar_git();

	while (true)
	{
		IPcam.read(orijinal);
		resize(orijinal, orijinal, Size(640, 480));
		GaussianBlur(orijinal, gri, Size(3, 3), 1, 1);

		inRange(gri, Scalar(mavi2, yesil2, kirmizi2), Scalar(mavi1, yesil1, kirmizi1), gri);

		imshow("Renk Ayar Penceresi", orijinal);
		imshow("Renk Ayar Penceresi Gri", gri);
		if (waitKey(100) == 13)
		{
			cout << "mavi1: " << mavi1 << endl << "yesil1: " << yesil1 << endl << "kirmizi1: " << kirmizi2 << endl;
			cout << "mavi2: " << mavi2 << endl << "yesil2: " << yesil2 << endl << "kirmizi2: " << kirmizi2 << endl;
			destroyWindow("Renk Ayar Penceresi");
			destroyWindow("Renk Ayar Penceresi Gri");
			destroyWindow(t_penceresi);
			islem_sec_ekrani();
		}

	}
}

void robot_kontrol()
{

	int tespit_edilemeyen_yuz = 0,tespit_edilemeyen_goz=0;
	

	vector<Rect>YuzTespiti;

	string pc_kam_fps, ip_kam_fps, ekran1 = "Bilgisayar Kamerasý", ekran2 = "IP Kamera";
	string pc_kam_x;

	CascadeClassifier yuzTespiti;	yuzTespiti.load(CaseCadeYuz);

	vector<vector<Point>>kontur;
	vector<Vec4i>hiy;

	CascadeClassifier gozBulma;
	gozBulma.load(CaseCadeGoz);

	CascadeClassifier gozBulma_sag;
	gozBulma_sag.load(CaseCadeGoz_sag);

	vector<vector<Point>>sagGozVek;
	vector<Vec4i>hiy_sagGoz;


	vector<vector<Point>>solGozVek;
	vector<Vec4i>hiy_solGoz;

	vector<Rect>bulunanGoz;
	vector<Rect>bulunanGoz_sag;

	vector<Vec3f>cemberTespit;


	namedWindow(ekran1, WINDOW_AUTOSIZE);
	namedWindow(ekran2, WINDOW_AUTOSIZE);

	Bilgisayar_Kamerasý_Ayarlari();

	while (true)
	{


		IPcam.read(ip_kam);
		ip_kam_fps = to_string((int)IPcam.get(CAP_PROP_FPS));
		resize(ip_kam, ip_kam, Size(640, 480));
		GaussianBlur(ip_kam, ip_kam_gri, Size(3, 3), 1, 1);
		inRange(ip_kam_gri, Scalar(mavi2, yesil2, kirmizi2), Scalar(mavi1, yesil1, kirmizi1), ip_kam_gri);
		findContours(ip_kam_gri, kontur, RETR_EXTERNAL , CHAIN_APPROX_NONE );
		
		if (kontur.size() > 0)
		{
			Moments a;
			Point2f center;
			float rad;
			a = moments(kontur.at(0));
			Point merkez(0, 0);
			merkez.x = a.m10 / a.m00;
			merkez.y = a.m01 / a.m00;
			drawContours(ip_kam, kontur, 0, Scalar(255, 0, 255), 5);
			circle(ip_kam, merkez, 0, Scalar(255, 255, 255), 5);
			string alan = to_string((float)contourArea(kontur.at(0)));
			string kordinat = "X: " + to_string(merkez.x) + " Y: " + to_string(merkez.y);
			putText(ip_kam, "Alan:" + alan, Point(30, 50), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1);
			putText(ip_kam, "Kordinat:" + kordinat, Point(30, 75), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1);
		}
		PCcam.read(bilgisayar_kam);
		pc_kam_fps = to_string((int)PCcam.get(CAP_PROP_FPS));
		putText(bilgisayar_kam, "FPS:" + pc_kam_fps, Point(30, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1);
		putText(ip_kam, "FPS:" + ip_kam_fps, Point(30, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1);

		cvtColor(bilgisayar_kam, bilgisayar_kam_gri, COLOR_BGR2GRAY);


		yuzTespiti.detectMultiScale(bilgisayar_kam_gri, YuzTespiti);
		if (YuzTespiti.size() > 0)
		{
			Mat yuzuCek(bilgisayar_kam, Rect(YuzTespiti.at(0).x, YuzTespiti.at(0).y, YuzTespiti.at(0).width, YuzTespiti.at(0).height / 1.5));

			cvtColor(yuzuCek, yuzuCek, COLOR_BGR2GRAY);

			gozBulma.detectMultiScale(yuzuCek, bulunanGoz);
			gozBulma_sag.detectMultiScale(yuzuCek, bulunanGoz_sag);

			GaussianBlur(yuzuCek, yuzuCek, Size(3, 3), 1);

			if (bulunanGoz.size() > 0  || bulunanGoz_sag.size() > 0)
			{
					tespit_edilemeyen_goz = 0;
				    //rectangle(yuzuCek, bulunanGoz.at(0), Scalar(255, 0, 0), 2);
					
					if (bulunanGoz.size() > 0)
					{
						Mat solGoz_t;
						Mat solGoz(yuzuCek, bulunanGoz.at(0));
						inRange(solGoz, Scalar(mavi2_gl, yesil2_gl, kirmizi2_gl), Scalar(mavi1_gl, yesil1_gl, kirmizi1_gl), solGoz_t);

						findContours(solGoz_t, solGozVek, RETR_EXTERNAL, CHAIN_APPROX_NONE);
						
						drawContours(solGoz, solGozVek, 0, Scalar(120, 120, 120), 2, 8);
						imshow("solGoz", solGoz);

						bulunanGoz.clear();
					}
				
					if (bulunanGoz_sag.size() > 0)
					{
						Mat sagGoz(yuzuCek, bulunanGoz_sag.at(0));
						Mat sagGoz_t;
						inRange(sagGoz, Scalar(mavi2_gr, yesil2_gr, kirmizi2_gr), Scalar(mavi1_gr, yesil1_gr, kirmizi1_gr), sagGoz_t);
						findContours(sagGoz_t, sagGozVek, RETR_EXTERNAL, CHAIN_APPROX_NONE);
						drawContours(sagGoz, sagGozVek, 0, Scalar(120, 120, 120), 2, 8);
						imshow("sagGoz", sagGoz);
						bulunanGoz_sag.clear();
					}
			}
			else
			{
				tespit_edilemeyen_goz++;

				putText(bilgisayar_kam, "Tespit edilemeyen goz sayisi:" + to_string(tespit_edilemeyen_goz), Point(30, 50), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1);

				if (tespit_edilemeyen_goz == 50)
				{
					cout << "Kontrol Durduruldu(Goz)" << endl;
					/*
					* Bu kýsma haberleþme protokolu gelicek
					*/
					tespit_edilemeyen_goz = 0;

				}

				destroyWindow("sagGoz");
				destroyWindow("solGoz");
			}
			rectangle(bilgisayar_kam, YuzTespiti.at(0), Scalar(0, 0, 255), 2);
			tespit_edilemeyen_yuz = 0;	
		}
		else
		{
			tespit_edilemeyen_yuz++;
			cout <<"Tespit edilemeyen yuz sayisi:" << tespit_edilemeyen_yuz << endl;
			putText(bilgisayar_kam, "Tespit edilemeyen yuz sayisi:" + to_string(tespit_edilemeyen_yuz), Point(30, 30), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1);
			if (tespit_edilemeyen_yuz == 20)
			{
				//yani yaklaþýk 10 sn yüz tespiti yapýlamaz ise  
				tespit_edilemeyen_yuz = 0;
				/*
				* bu kýsma haberleþme protokolü eklenecek
				*/
				cout << "Kontrol Durduruldu!!" << endl;
			}
		}

		imshow(ekran1, bilgisayar_kam);
		imshow(ekran2, ip_kam);
		imshow("ip kam gri", ip_kam_gri);

		if (waitKey(100) == 27)
			break;
	}

}

void goz_algilama()
{
	cout << "Lutfen goz algilama parametrelerini kaydedizi" << endl;

	TrackBar_git();
	Bilgisayar_Kamerasý_Ayarlari();

	Mat goz1, goz2, goz1_gri, goz2_gri,solGoz_t,sagGoz_t;
	CascadeClassifier gozBulma;
	gozBulma.load(CaseCadeGoz);

	CascadeClassifier gozBulma_sag;
	gozBulma_sag.load(CaseCadeGoz_sag);

	CascadeClassifier yuzBulma;
	yuzBulma.load(CaseCadeYuz);

	vector<Rect>bulunanGoz;
	vector<Rect>bulunanYuz;
	vector<Rect>bulunanGoz_sag;

	Mat yuzuCek_orj;

	while (true)
	{
		PCcam.read(bilgisayar_kam);
		cvtColor(bilgisayar_kam, bilgisayar_kam_gri, COLOR_BGR2GRAY);
		yuzBulma.detectMultiScale(bilgisayar_kam_gri, bulunanYuz);
		if (bulunanYuz.size() > 0)
		{
			Mat yuzuCek(bilgisayar_kam, Rect(bulunanYuz.at(0).x ,bulunanYuz.at(0).y,bulunanYuz.at(0).width,bulunanYuz.at(0).height / 1.5));
			yuzuCek.copyTo(yuzuCek_orj);
			
			cvtColor(yuzuCek, yuzuCek, COLOR_BGR2GRAY);

			gozBulma.detectMultiScale(yuzuCek, bulunanGoz);
			gozBulma_sag.detectMultiScale(yuzuCek, bulunanGoz_sag);

			GaussianBlur(yuzuCek, yuzuCek, Size(3, 3), 1);
			
			if (bulunanGoz.size()>0)
			{

				//rectangle(yuzuCek, bulunanGoz.at(0), Scalar(255, 0, 0), 2);
				if (sayac==0)
				{
					Mat solGoz(yuzuCek, bulunanGoz.at(0));

					inRange(solGoz, Scalar(mavi2, yesil2, kirmizi2), Scalar(mavi1, yesil1, kirmizi1), solGoz_t);

					resize(solGoz_t, solGoz_t, Size(640, 480));
					imshow("Sol Goz", solGoz);
					imshow("Sol Goz T", solGoz_t);
					
				}
				
			}
		
			if (bulunanGoz_sag.size() > 0 && sayac == 1)
			{
			//	rectangle(yuzuCek, bulunanGoz_sag.at(0), Scalar(0, 0, 0), 2);

				Mat sagGoz(yuzuCek, bulunanGoz_sag.at(0));
				inRange(sagGoz, Scalar(mavi2, yesil2, kirmizi2), Scalar(mavi1, yesil1, kirmizi1), sagGoz_t);
				resize(sagGoz_t, sagGoz_t, Size(640, 480));
				imshow("Sag Goz", sagGoz);
				imshow("Sag Goz T", sagGoz_t);
			}
	
			imshow("cekilen Yuz", yuzuCek);

		}

		imshow("Bilgisayar kamerasi", bilgisayar_kam_gri);
		imshow("Bilgisayar kamerasi2", bilgisayar_kam);
		if (waitKey(100) == 13)
		{
			if (sayac == 0)
			{
				sayac++;
				mavi1_gl = mavi1;  yesil1_gl = yesil1; kirmizi1_gl = kirmizi1;
				mavi2_gl = mavi2;  yesil2_gl = yesil2; kirmizi2_gl = kirmizi2;
				destroyWindow("Sol Goz");
				destroyWindow("Sol Goz T");

				
				cout << "mavi1_gl:" << mavi1_gl << "  " << "yesil1_gl:" << yesil1_gl << "  " << "kirmizi1_gl:" << kirmizi1_gl << endl<<endl;
				cout << "mavi2_gl:" << mavi2_gl << "  " << "yesil2_gr:" << yesil2_gl << "  " << "kirmizi2_gl:" << kirmizi2_gl << endl<<endl;
				
				cout << "\n\nSol Goz parametreleri l-kaydedildi\n\n" << endl;
			}
			else if (sayac == 1)
			{	
				mavi1_gr = mavi1;  yesil1_gr = yesil1; kirmizi1_gr = kirmizi1;
				mavi2_gr = mavi2;  yesil2_gr = yesil2; kirmizi2_gr = kirmizi2;
			
				cout << "mavi1_gr:" << mavi1_gr <<"  " << "yesil1_gr:" << yesil1_gr << "   " << "kirmizi1_gr:" << kirmizi1_gr << endl<<endl;
				cout << "mavi2_gr:" << mavi2_gr << "  " << "yesil2_gr:" << yesil2_gr << "  " << "kirmizi2:" << kirmizi2_gr << endl<<endl;
				cout << "\n\nSag goz parametreleri kaydedildi\n\n" << endl;
				destroyAllWindows();
				sayac = 0;
				
				break;
			}
	
		}

	}
	
}

void islem_sec_ekrani()
{
	int secilen_islem;
	cout << "\n\t********  Duzce Universitesi Biyomedikal Muhendisligi  ******** \n\n";
	cout << "\t********** Lutfen Yapmak Istediginiz Islemi Seciniz **********\n" << endl;
	cout << "1-Renk Araligi Tespiti" << endl;
	cout << "2-Robot Konrol\n" << endl;
	cout << "Isleminiz:";

	cin >> secilen_islem;

	switch (secilen_islem)
	{
	case 1:
		renk_tespit_degeri_bul();
		break;
	case 2:
	
		Bilgisayar_Kamerasý_Ayarlari();
		IP_Kamera_Ayarlari();
		robot_kontrol();
		break;
	default:
		cout << "Yanlis Tuslama!!" << endl;
	}

}
int main()
{
	cout << "\n\t********  Duzce Universitesi Biyomedikal Muhendisligi  ******** \n\n";
	cout << "IP Adresi Giriniz:";
	cin >> IP_Adresi;

	cout << "\n\nLutfen goz algilama parametrelerini ayarlayiniz!!\n\n" << endl;
	goz_algilama();

	islem_sec_ekrani();

	return 0;

}

