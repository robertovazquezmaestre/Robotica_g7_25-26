/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/specificworker.h"
#include <QtGui/qtextcursor.h>
#include <QScreen>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_SpecificWorker_t {
    uint offsetsAndSizes[32];
    char stringdata0[15];
    char stringdata1[11];
    char stringdata2[1];
    char stringdata3[8];
    char stringdata4[10];
    char stringdata5[8];
    char stringdata6[14];
    char stringdata7[16];
    char stringdata8[2];
    char stringdata9[22];
    char stringdata10[11];
    char stringdata11[40];
    char stringdata12[12];
    char stringdata13[7];
    char stringdata14[14];
    char stringdata15[15];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_SpecificWorker_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_SpecificWorker_t qt_meta_stringdata_SpecificWorker = {
    {
        QT_MOC_LITERAL(0, 14),  // "SpecificWorker"
        QT_MOC_LITERAL(15, 10),  // "initialize"
        QT_MOC_LITERAL(26, 0),  // ""
        QT_MOC_LITERAL(27, 7),  // "compute"
        QT_MOC_LITERAL(35, 9),  // "emergency"
        QT_MOC_LITERAL(45, 7),  // "restore"
        QT_MOC_LITERAL(53, 13),  // "startup_check"
        QT_MOC_LITERAL(67, 15),  // "new_target_slot"
        QT_MOC_LITERAL(83, 1),  // "p"
        QT_MOC_LITERAL(85, 21),  // "update_ropot_position"
        QT_MOC_LITERAL(107, 10),  // "chocachoca"
        QT_MOC_LITERAL(118, 39),  // "std::optional<RoboCompLidar3D..."
        QT_MOC_LITERAL(158, 11),  // "filter_data"
        QT_MOC_LITERAL(170, 6),  // "float&"
        QT_MOC_LITERAL(177, 13),  // "advance_speed"
        QT_MOC_LITERAL(191, 14)   // "rotation_speed"
    },
    "SpecificWorker",
    "initialize",
    "",
    "compute",
    "emergency",
    "restore",
    "startup_check",
    "new_target_slot",
    "p",
    "update_ropot_position",
    "chocachoca",
    "std::optional<RoboCompLidar3D::TPoints>",
    "filter_data",
    "float&",
    "advance_speed",
    "rotation_speed"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_SpecificWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   62,    2, 0x0a,    1 /* Public */,
       3,    0,   63,    2, 0x0a,    2 /* Public */,
       4,    0,   64,    2, 0x0a,    3 /* Public */,
       5,    0,   65,    2, 0x0a,    4 /* Public */,
       6,    0,   66,    2, 0x0a,    5 /* Public */,
       7,    1,   67,    2, 0x0a,    6 /* Public */,
       9,    0,   70,    2, 0x0a,    8 /* Public */,
      10,    3,   71,    2, 0x0a,    9 /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Int,
    QMetaType::Void, QMetaType::QPointF,    8,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 11, 0x80000000 | 13, 0x80000000 | 13,   12,   14,   15,

       0        // eod
};

Q_CONSTINIT const QMetaObject SpecificWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<GenericWorker::staticMetaObject>(),
    qt_meta_stringdata_SpecificWorker.offsetsAndSizes,
    qt_meta_data_SpecificWorker,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_SpecificWorker_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<SpecificWorker, std::true_type>,
        // method 'initialize'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'compute'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'emergency'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'restore'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'startup_check'
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'new_target_slot'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QPointF, std::false_type>,
        // method 'update_ropot_position'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'chocachoca'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<std::optional<RoboCompLidar3D::TPoints>, std::false_type>,
        QtPrivate::TypeAndForceComplete<float &, std::false_type>,
        QtPrivate::TypeAndForceComplete<float &, std::false_type>
    >,
    nullptr
} };

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SpecificWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->initialize(); break;
        case 1: _t->compute(); break;
        case 2: _t->emergency(); break;
        case 3: _t->restore(); break;
        case 4: { int _r = _t->startup_check();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 5: _t->new_target_slot((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1]))); break;
        case 6: _t->update_ropot_position(); break;
        case 7: _t->chocachoca((*reinterpret_cast< std::add_pointer_t<std::optional<RoboCompLidar3D::TPoints>>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float&>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float&>>(_a[3]))); break;
        default: ;
        }
    }
}

const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker.stringdata0))
        return static_cast<void*>(this);
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 8;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
