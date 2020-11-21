.class public interface abstract Lsensor_msgs/Range;
.super Ljava/lang/Object;
.source "Range.java"

# interfaces
.implements Lorg/ros/internal/message/Message;


# static fields
.field public static final INFRARED:B = 0x1t

.field public static final ULTRASOUND:B = 0x0t

.field public static final _DEFINITION:Ljava/lang/String; = "# Single range reading from an active ranger that emits energy and reports\n# one range reading that is valid along an arc at the distance measured. \n# This message is  not appropriate for laser scanners. See the LaserScan\n# message if you are working with a laser scanner.\n\n# This message also can represent a fixed-distance (binary) ranger.  This\n# sensor will have min_range===max_range===distance of detection.\n# These sensors follow REP 117 and will output -Inf if the object is detected\n# and +Inf if the object is outside of the detection range.\n\nHeader header           # timestamp in the header is the time the ranger\n                        # returned the distance reading\n\n# Radiation type enums\n# If you want a value added to this list, send an email to the ros-users list\nuint8 ULTRASOUND=0\nuint8 INFRARED=1\n\nuint8 radiation_type    # the type of radiation used by the sensor\n                        # (sound, IR, etc) [enum]\n\nfloat32 field_of_view   # the size of the arc that the distance reading is\n                        # valid for [rad]\n                        # the object causing the range reading may have\n                        # been anywhere within -field_of_view/2 and\n                        # field_of_view/2 at the measured range. \n                        # 0 angle corresponds to the x-axis of the sensor.\n\nfloat32 min_range       # minimum range value [m]\nfloat32 max_range       # maximum range value [m]\n                        # Fixed distance rangers require min_range==max_range\n\nfloat32 range           # range data [m]\n                        # (Note: values < range_min or > range_max\n                        # should be discarded)\n                        # Fixed distance rangers only output -Inf or +Inf.\n                        # -Inf represents a detection within fixed distance.\n                        # (Detection too close to the sensor to quantify)\n                        # +Inf represents no detection within the fixed distance.\n                        # (Object out of range)"

.field public static final _TYPE:Ljava/lang/String; = "sensor_msgs/Range"


# virtual methods
.method public abstract getFieldOfView()F
.end method

.method public abstract getHeader()Lstd_msgs/Header;
.end method

.method public abstract getMaxRange()F
.end method

.method public abstract getMinRange()F
.end method

.method public abstract getRadiationType()B
.end method

.method public abstract getRange()F
.end method

.method public abstract setFieldOfView(F)V
.end method

.method public abstract setHeader(Lstd_msgs/Header;)V
.end method

.method public abstract setMaxRange(F)V
.end method

.method public abstract setMinRange(F)V
.end method

.method public abstract setRadiationType(B)V
.end method

.method public abstract setRange(F)V
.end method
