/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "viewer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_4;
    Viewer *CloudViewer;
    QTabWidget *infoTab;
    QWidget *choose;
    QWidget *layoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_3;
    QCheckBox *cloudCB;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label;
    QSpacerItem *horizontalSpacer;
    QCheckBox *groundCB;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_2;
    QCheckBox *obstacleCB;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_8;
    QSpacerItem *horizontalSpacer_6;
    QCheckBox *clusterCB;
    QHBoxLayout *horizontalLayout;
    QLabel *label_10;
    QSpacerItem *horizontalSpacer_8;
    QCheckBox *depthClusterCB;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_4;
    QCheckBox *insertCB;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_5;
    QSpacerItem *horizontalSpacer_5;
    QCheckBox *lineCB;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_11;
    QSpacerItem *horizontalSpacer_9;
    QCheckBox *bboxCB;
    QHBoxLayout *horizontalLayout_14;
    QLabel *label_12;
    QSpacerItem *horizontalSpacer_10;
    QCheckBox *voxelCB;
    QHBoxLayout *horizontalLayout_15;
    QLabel *label_14;
    QSpacerItem *horizontalSpacer_11;
    QCheckBox *guassCircleCB;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_9;
    QSpacerItem *horizontalSpacer_7;
    QSpinBox *girdNumSB;
    QWidget *tab;
    QWidget *layoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_6;
    QSpinBox *paramSB;
    QLabel *label_7;
    QDoubleSpinBox *paramDSB;
    QPlainTextEdit *paramPTE;
    QHBoxLayout *horizontalLayout_13;
    QPushButton *updatePB;
    QLabel *label_13;
    QComboBox *ObjectSelectCB;
    QPushButton *clearSelectionPB;
    QPushButton *resetBT;
    QPushButton *quit;
    QHBoxLayout *horizontalLayout_3;
    QSpinBox *dataSeqSB;
    QPushButton *openFolderBT;
    QPushButton *playBT;
    QSpinBox *DataIdxSBox;
    QSlider *DataIdxVSlider;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(986, 703);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout_2 = new QVBoxLayout(centralWidget);
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(0);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        CloudViewer = new Viewer(centralWidget);
        CloudViewer->setObjectName(QStringLiteral("CloudViewer"));
        CloudViewer->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(CloudViewer->sizePolicy().hasHeightForWidth());
        CloudViewer->setSizePolicy(sizePolicy);
        CloudViewer->setMouseTracking(false);

        horizontalLayout_4->addWidget(CloudViewer);

        infoTab = new QTabWidget(centralWidget);
        infoTab->setObjectName(QStringLiteral("infoTab"));
        infoTab->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(infoTab->sizePolicy().hasHeightForWidth());
        infoTab->setSizePolicy(sizePolicy1);
        infoTab->setMinimumSize(QSize(280, 0));
        QFont font;
        font.setPointSize(9);
        infoTab->setFont(font);
        choose = new QWidget();
        choose->setObjectName(QStringLiteral("choose"));
        choose->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
        layoutWidget_3 = new QWidget(choose);
        layoutWidget_3->setObjectName(QStringLiteral("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(0, 0, 289, 311));
        verticalLayout_3 = new QVBoxLayout(layoutWidget_3);
        verticalLayout_3->setSpacing(5);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(0);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        label_3 = new QLabel(layoutWidget_3);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_6->addWidget(label_3);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_3);

        cloudCB = new QCheckBox(layoutWidget_3);
        cloudCB->setObjectName(QStringLiteral("cloudCB"));

        horizontalLayout_6->addWidget(cloudCB);


        verticalLayout_3->addLayout(horizontalLayout_6);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label = new QLabel(layoutWidget_3);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_5->addWidget(label);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);

        groundCB = new QCheckBox(layoutWidget_3);
        groundCB->setObjectName(QStringLiteral("groundCB"));

        horizontalLayout_5->addWidget(groundCB);


        verticalLayout_3->addLayout(horizontalLayout_5);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_2 = new QLabel(layoutWidget_3);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_7->addWidget(label_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_2);

        obstacleCB = new QCheckBox(layoutWidget_3);
        obstacleCB->setObjectName(QStringLiteral("obstacleCB"));

        horizontalLayout_7->addWidget(obstacleCB);


        verticalLayout_3->addLayout(horizontalLayout_7);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        label_8 = new QLabel(layoutWidget_3);
        label_8->setObjectName(QStringLiteral("label_8"));

        horizontalLayout_10->addWidget(label_8);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_6);

        clusterCB = new QCheckBox(layoutWidget_3);
        clusterCB->setObjectName(QStringLiteral("clusterCB"));

        horizontalLayout_10->addWidget(clusterCB);


        verticalLayout_3->addLayout(horizontalLayout_10);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_10 = new QLabel(layoutWidget_3);
        label_10->setObjectName(QStringLiteral("label_10"));

        horizontalLayout->addWidget(label_10);

        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_8);

        depthClusterCB = new QCheckBox(layoutWidget_3);
        depthClusterCB->setObjectName(QStringLiteral("depthClusterCB"));

        horizontalLayout->addWidget(depthClusterCB);


        verticalLayout_3->addLayout(horizontalLayout);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        label_4 = new QLabel(layoutWidget_3);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_8->addWidget(label_4);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_4);

        insertCB = new QCheckBox(layoutWidget_3);
        insertCB->setObjectName(QStringLiteral("insertCB"));

        horizontalLayout_8->addWidget(insertCB);


        verticalLayout_3->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        label_5 = new QLabel(layoutWidget_3);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_9->addWidget(label_5);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer_5);

        lineCB = new QCheckBox(layoutWidget_3);
        lineCB->setObjectName(QStringLiteral("lineCB"));

        horizontalLayout_9->addWidget(lineCB);


        verticalLayout_3->addLayout(horizontalLayout_9);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_11 = new QLabel(layoutWidget_3);
        label_11->setObjectName(QStringLiteral("label_11"));

        horizontalLayout_2->addWidget(label_11);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_9);

        bboxCB = new QCheckBox(layoutWidget_3);
        bboxCB->setObjectName(QStringLiteral("bboxCB"));

        horizontalLayout_2->addWidget(bboxCB);


        verticalLayout_3->addLayout(horizontalLayout_2);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setObjectName(QStringLiteral("horizontalLayout_14"));
        label_12 = new QLabel(layoutWidget_3);
        label_12->setObjectName(QStringLiteral("label_12"));

        horizontalLayout_14->addWidget(label_12);

        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_14->addItem(horizontalSpacer_10);

        voxelCB = new QCheckBox(layoutWidget_3);
        voxelCB->setObjectName(QStringLiteral("voxelCB"));

        horizontalLayout_14->addWidget(voxelCB);


        verticalLayout_3->addLayout(horizontalLayout_14);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setObjectName(QStringLiteral("horizontalLayout_15"));
        label_14 = new QLabel(layoutWidget_3);
        label_14->setObjectName(QStringLiteral("label_14"));

        horizontalLayout_15->addWidget(label_14);

        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_15->addItem(horizontalSpacer_11);

        guassCircleCB = new QCheckBox(layoutWidget_3);
        guassCircleCB->setObjectName(QStringLiteral("guassCircleCB"));

        horizontalLayout_15->addWidget(guassCircleCB);


        verticalLayout_3->addLayout(horizontalLayout_15);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QStringLiteral("horizontalLayout_11"));
        label_9 = new QLabel(layoutWidget_3);
        label_9->setObjectName(QStringLiteral("label_9"));

        horizontalLayout_11->addWidget(label_9);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer_7);

        girdNumSB = new QSpinBox(layoutWidget_3);
        girdNumSB->setObjectName(QStringLiteral("girdNumSB"));

        horizontalLayout_11->addWidget(girdNumSB);


        verticalLayout_3->addLayout(horizontalLayout_11);

        infoTab->addTab(choose, QString());
        layoutWidget_3->raise();
        voxelCB->raise();
        label_12->raise();
        label_14->raise();
        guassCircleCB->raise();
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        layoutWidget_4 = new QWidget(tab);
        layoutWidget_4->setObjectName(QStringLiteral("layoutWidget_4"));
        layoutWidget_4->setGeometry(QRect(0, 0, 281, 961));
        verticalLayout_4 = new QVBoxLayout(layoutWidget_4);
        verticalLayout_4->setSpacing(0);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setSizeConstraint(QLayout::SetMaximumSize);
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        horizontalLayout_12->setSizeConstraint(QLayout::SetMaximumSize);
        label_6 = new QLabel(layoutWidget_4);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_12->addWidget(label_6);

        paramSB = new QSpinBox(layoutWidget_4);
        paramSB->setObjectName(QStringLiteral("paramSB"));

        horizontalLayout_12->addWidget(paramSB);

        label_7 = new QLabel(layoutWidget_4);
        label_7->setObjectName(QStringLiteral("label_7"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy2);

        horizontalLayout_12->addWidget(label_7);

        paramDSB = new QDoubleSpinBox(layoutWidget_4);
        paramDSB->setObjectName(QStringLiteral("paramDSB"));
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(paramDSB->sizePolicy().hasHeightForWidth());
        paramDSB->setSizePolicy(sizePolicy3);

        horizontalLayout_12->addWidget(paramDSB);


        verticalLayout_4->addLayout(horizontalLayout_12);

        paramPTE = new QPlainTextEdit(layoutWidget_4);
        paramPTE->setObjectName(QStringLiteral("paramPTE"));

        verticalLayout_4->addWidget(paramPTE);

        infoTab->addTab(tab, QString());

        horizontalLayout_4->addWidget(infoTab);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QStringLiteral("horizontalLayout_13"));
        updatePB = new QPushButton(centralWidget);
        updatePB->setObjectName(QStringLiteral("updatePB"));

        horizontalLayout_13->addWidget(updatePB);

        label_13 = new QLabel(centralWidget);
        label_13->setObjectName(QStringLiteral("label_13"));
        QSizePolicy sizePolicy4(QSizePolicy::Maximum, QSizePolicy::Preferred);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(label_13->sizePolicy().hasHeightForWidth());
        label_13->setSizePolicy(sizePolicy4);

        horizontalLayout_13->addWidget(label_13);

        ObjectSelectCB = new QComboBox(centralWidget);
        ObjectSelectCB->setObjectName(QStringLiteral("ObjectSelectCB"));

        horizontalLayout_13->addWidget(ObjectSelectCB);

        clearSelectionPB = new QPushButton(centralWidget);
        clearSelectionPB->setObjectName(QStringLiteral("clearSelectionPB"));

        horizontalLayout_13->addWidget(clearSelectionPB);

        resetBT = new QPushButton(centralWidget);
        resetBT->setObjectName(QStringLiteral("resetBT"));

        horizontalLayout_13->addWidget(resetBT);

        quit = new QPushButton(centralWidget);
        quit->setObjectName(QStringLiteral("quit"));

        horizontalLayout_13->addWidget(quit);


        verticalLayout->addLayout(horizontalLayout_13);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        dataSeqSB = new QSpinBox(centralWidget);
        dataSeqSB->setObjectName(QStringLiteral("dataSeqSB"));

        horizontalLayout_3->addWidget(dataSeqSB);

        openFolderBT = new QPushButton(centralWidget);
        openFolderBT->setObjectName(QStringLiteral("openFolderBT"));

        horizontalLayout_3->addWidget(openFolderBT);

        playBT = new QPushButton(centralWidget);
        playBT->setObjectName(QStringLiteral("playBT"));

        horizontalLayout_3->addWidget(playBT);

        DataIdxSBox = new QSpinBox(centralWidget);
        DataIdxSBox->setObjectName(QStringLiteral("DataIdxSBox"));

        horizontalLayout_3->addWidget(DataIdxSBox);

        DataIdxVSlider = new QSlider(centralWidget);
        DataIdxVSlider->setObjectName(QStringLiteral("DataIdxVSlider"));
        DataIdxVSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(DataIdxVSlider);


        verticalLayout->addLayout(horizontalLayout_3);


        verticalLayout_2->addLayout(verticalLayout);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);
        QObject::connect(quit, SIGNAL(clicked()), MainWindow, SLOT(close()));
        QObject::connect(DataIdxVSlider, SIGNAL(valueChanged(int)), DataIdxSBox, SLOT(setValue(int)));
        QObject::connect(DataIdxSBox, SIGNAL(valueChanged(int)), DataIdxVSlider, SLOT(setValue(int)));

        infoTab->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "              orgin   ", Q_NULLPTR));
        cloudCB->setText(QApplication::translate("MainWindow", "    cloud      ", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "           Ground", Q_NULLPTR));
        groundCB->setText(QApplication::translate("MainWindow", "  ground     ", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "           obstacle", Q_NULLPTR));
        obstacleCB->setText(QApplication::translate("MainWindow", "obstacle    ", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "girdCluster", Q_NULLPTR));
        clusterCB->setText(QApplication::translate("MainWindow", " cluster      ", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindow", "depthCluster", Q_NULLPTR));
        depthClusterCB->setText(QApplication::translate("MainWindow", " cluster      ", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "           insert    ", Q_NULLPTR));
        insertCB->setText(QApplication::translate("MainWindow", "    insert     ", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "            line      ", Q_NULLPTR));
        lineCB->setText(QApplication::translate("MainWindow", "      line       ", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindow", "          bbox     ", Q_NULLPTR));
        bboxCB->setText(QApplication::translate("MainWindow", "     bbox      ", Q_NULLPTR));
        label_12->setText(QApplication::translate("MainWindow", "        voxel     ", Q_NULLPTR));
        voxelCB->setText(QApplication::translate("MainWindow", "    voxels    ", Q_NULLPTR));
        label_14->setText(QApplication::translate("MainWindow", "   gaussCircle", Q_NULLPTR));
        guassCircleCB->setText(QApplication::translate("MainWindow", "guassCircle", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "       gridNum    ", Q_NULLPTR));
        infoTab->setTabText(infoTab->indexOf(choose), QApplication::translate("MainWindow", "\351\241\265", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "paramID", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "value", Q_NULLPTR));
        paramPTE->setPlainText(QApplication::translate("MainWindow", "groundRemove :                                           ID\n"
"line_search_angle(2)                                   0\n"
"max_slope(0.35)                                           1\n"
"tHmin(-2.15)                                                   2\n"
"tHmax(1.0)                                                      3\n"
"tHDiff(0.2)                                                       4\n"
"hSensor(1.73)                                                 5\n"
"r_min_bin(0.05)                                              6\n"
"r_max_bin(2)                                                   7\n"
"r_min_square(3 * 3)                                      8\n"
"r_max_square(120 * 120)                            9\n"
"n_bins(120)                                                     10\n"
"n_segments(240)                                          11\n"
"null                                                                    12\n"
"max_dist_to_line(0.15)                               13\n"
"visualize                  "
                        "                                         14\n"
"max_error_square(0.01)                             15\n"
"long_threshold(2.0)                                      16\n"
"max_long_height(0.2)                                  17\n"
"max_start_height(0.3)                                 18\n"
"sensor_height(1.70)                                     19\n"
"null                                                                     20\n"
"null                                                                     21\n"
"min_split_dist(0.1)                                         22\n"
"theta_start(65.1277)                                     23\n"
"theta_end(2)                                                    24\n"
"angle_resolution(0.41)                                 25\n"
"applayMedianFilterMinZ(0)                         26\n"
"\n"
"depthCluster:\n"
"filter(1)                                                                 27\n"
"angle_threshold(10)                                        28\n"
"gridImageResize(1) "
                        "                                         29\n"
"", Q_NULLPTR));
        infoTab->setTabText(infoTab->indexOf(tab), QApplication::translate("MainWindow", "\351\241\265", Q_NULLPTR));
        updatePB->setText(QApplication::translate("MainWindow", "update", Q_NULLPTR));
        label_13->setText(QApplication::translate("MainWindow", "Object:", Q_NULLPTR));
        clearSelectionPB->setText(QApplication::translate("MainWindow", "clearSelection", Q_NULLPTR));
        resetBT->setText(QApplication::translate("MainWindow", "reset", Q_NULLPTR));
        quit->setText(QApplication::translate("MainWindow", "quit", Q_NULLPTR));
        openFolderBT->setText(QApplication::translate("MainWindow", "Open Folder", Q_NULLPTR));
        playBT->setText(QApplication::translate("MainWindow", "Play", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
