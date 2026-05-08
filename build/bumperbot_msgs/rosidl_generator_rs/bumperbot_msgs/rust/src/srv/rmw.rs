#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "bumperbot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__bumperbot_msgs__srv__AddTwoInts_Request() -> *const std::ffi::c_void;
}

#[link(name = "bumperbot_msgs__rosidl_generator_c")]
extern "C" {
    fn bumperbot_msgs__srv__AddTwoInts_Request__init(msg: *mut AddTwoInts_Request) -> bool;
    fn bumperbot_msgs__srv__AddTwoInts_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Request>, size: usize) -> bool;
    fn bumperbot_msgs__srv__AddTwoInts_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Request>);
    fn bumperbot_msgs__srv__AddTwoInts_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AddTwoInts_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Request>) -> bool;
}

// Corresponds to bumperbot_msgs__srv__AddTwoInts_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AddTwoInts_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub a: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub b: i64,

}



impl Default for AddTwoInts_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !bumperbot_msgs__srv__AddTwoInts_Request__init(&mut msg as *mut _) {
        panic!("Call to bumperbot_msgs__srv__AddTwoInts_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AddTwoInts_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__AddTwoInts_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__AddTwoInts_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__AddTwoInts_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AddTwoInts_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AddTwoInts_Request where Self: Sized {
  const TYPE_NAME: &'static str = "bumperbot_msgs/srv/AddTwoInts_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__bumperbot_msgs__srv__AddTwoInts_Request() }
  }
}


#[link(name = "bumperbot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__bumperbot_msgs__srv__AddTwoInts_Response() -> *const std::ffi::c_void;
}

#[link(name = "bumperbot_msgs__rosidl_generator_c")]
extern "C" {
    fn bumperbot_msgs__srv__AddTwoInts_Response__init(msg: *mut AddTwoInts_Response) -> bool;
    fn bumperbot_msgs__srv__AddTwoInts_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Response>, size: usize) -> bool;
    fn bumperbot_msgs__srv__AddTwoInts_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Response>);
    fn bumperbot_msgs__srv__AddTwoInts_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AddTwoInts_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Response>) -> bool;
}

// Corresponds to bumperbot_msgs__srv__AddTwoInts_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AddTwoInts_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub sum: i64,

}



impl Default for AddTwoInts_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !bumperbot_msgs__srv__AddTwoInts_Response__init(&mut msg as *mut _) {
        panic!("Call to bumperbot_msgs__srv__AddTwoInts_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AddTwoInts_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__AddTwoInts_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__AddTwoInts_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__AddTwoInts_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AddTwoInts_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AddTwoInts_Response where Self: Sized {
  const TYPE_NAME: &'static str = "bumperbot_msgs/srv/AddTwoInts_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__bumperbot_msgs__srv__AddTwoInts_Response() }
  }
}


#[link(name = "bumperbot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__bumperbot_msgs__srv__GetTransform_Request() -> *const std::ffi::c_void;
}

#[link(name = "bumperbot_msgs__rosidl_generator_c")]
extern "C" {
    fn bumperbot_msgs__srv__GetTransform_Request__init(msg: *mut GetTransform_Request) -> bool;
    fn bumperbot_msgs__srv__GetTransform_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetTransform_Request>, size: usize) -> bool;
    fn bumperbot_msgs__srv__GetTransform_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetTransform_Request>);
    fn bumperbot_msgs__srv__GetTransform_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetTransform_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetTransform_Request>) -> bool;
}

// Corresponds to bumperbot_msgs__srv__GetTransform_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTransform_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub child_frame_id: rosidl_runtime_rs::String,

}



impl Default for GetTransform_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !bumperbot_msgs__srv__GetTransform_Request__init(&mut msg as *mut _) {
        panic!("Call to bumperbot_msgs__srv__GetTransform_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetTransform_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__GetTransform_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__GetTransform_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__GetTransform_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetTransform_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetTransform_Request where Self: Sized {
  const TYPE_NAME: &'static str = "bumperbot_msgs/srv/GetTransform_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__bumperbot_msgs__srv__GetTransform_Request() }
  }
}


#[link(name = "bumperbot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__bumperbot_msgs__srv__GetTransform_Response() -> *const std::ffi::c_void;
}

#[link(name = "bumperbot_msgs__rosidl_generator_c")]
extern "C" {
    fn bumperbot_msgs__srv__GetTransform_Response__init(msg: *mut GetTransform_Response) -> bool;
    fn bumperbot_msgs__srv__GetTransform_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetTransform_Response>, size: usize) -> bool;
    fn bumperbot_msgs__srv__GetTransform_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetTransform_Response>);
    fn bumperbot_msgs__srv__GetTransform_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetTransform_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetTransform_Response>) -> bool;
}

// Corresponds to bumperbot_msgs__srv__GetTransform_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetTransform_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub transform: geometry_msgs::msg::rmw::TransformStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for GetTransform_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !bumperbot_msgs__srv__GetTransform_Response__init(&mut msg as *mut _) {
        panic!("Call to bumperbot_msgs__srv__GetTransform_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetTransform_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__GetTransform_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__GetTransform_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { bumperbot_msgs__srv__GetTransform_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetTransform_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetTransform_Response where Self: Sized {
  const TYPE_NAME: &'static str = "bumperbot_msgs/srv/GetTransform_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__bumperbot_msgs__srv__GetTransform_Response() }
  }
}






#[link(name = "bumperbot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__bumperbot_msgs__srv__AddTwoInts() -> *const std::ffi::c_void;
}

// Corresponds to bumperbot_msgs__srv__AddTwoInts
#[allow(missing_docs, non_camel_case_types)]
pub struct AddTwoInts;

impl rosidl_runtime_rs::Service for AddTwoInts {
    type Request = AddTwoInts_Request;
    type Response = AddTwoInts_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__bumperbot_msgs__srv__AddTwoInts() }
    }
}




#[link(name = "bumperbot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__bumperbot_msgs__srv__GetTransform() -> *const std::ffi::c_void;
}

// Corresponds to bumperbot_msgs__srv__GetTransform
#[allow(missing_docs, non_camel_case_types)]
pub struct GetTransform;

impl rosidl_runtime_rs::Service for GetTransform {
    type Request = GetTransform_Request;
    type Response = GetTransform_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__bumperbot_msgs__srv__GetTransform() }
    }
}


