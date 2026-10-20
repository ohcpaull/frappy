from .rest import RestIO
from frappy.core import Module, HasIO, StringType, IntRange, FloatRange, ArrayOf, StructOf, Parameter, nopoll
from frappy.errors import CommunicationFailedError


class BalluffPort(HasIO, Module):
    ioClass = RestIO

    #ioClass.base_url = http://10.157.205.225/api/balluff/v1/

    port_number = Parameter('Port number on Balluff IO-Link', IntRange(1,10))
    device_alias = Parameter('Alias for Balluff port', StringType())
    identification = Parameter(
        'Connected device to Balluff port', 
        datatype=StructOf(
            vendor_id=IntRange(),
            device_id=IntRange(),
            device_alias=StringType(),
            vendor_name=StringType(),
            vendor_text=StringType(),
            product_name=StringType(),
            product_id=StringType(),
            product_text=StringType(),
            serial_number=StringType(),
            firmware_revision=StringType()
        ),
        readonly=True
    )

    @nopoll
    def read_identification(self):
        url = self.io._make_url(f'/devices/{self.device_alias}/identification')
        response = self.io.get(url)

        try:
            return {
                "vendor_id": int(response["vendorId"]),
                "device_id": int(response["deviceId"]),
                "device_alias": str(response["deviceAlias"]),
                "vendor_name": str(response["vendorName"]),
                "vendor_text": str(response["vendorText"]),
                "product_name": str(response["productName"]),
                "product_text": str(response["productText"]),
                "serial_number": str(response["serialNumber"]),
                "firmware_revision": str(response["firmwareRevision"]),
            }
        except(KeyError, TypeError, ValueError) as exc:
            raise CommunicationFailedError(
                f"Invalid IO-Link identifaction response: {response!r}"
            )


class BalluffIOLink(HasIO, Module):
    ioClass = RestIO

    port_count = Parameter('Number of IO-Link ports', datatype=IntRange(1,16), readonly=True)
    
    full_identification = Parameter(
        'Identification of all used ports on Balluff IO-Link',
        datatype=ArrayOf(
            StructOf(
            vendor_id=IntRange(),
            device_id=IntRange(),
            port_number = IntRange(),
            device_alias=StringType(),
            vendor_name=StringType(),
            vendor_text=StringType(),
            product_name=StringType(),
            product_id=StringType(),
            product_text=StringType(),
            serial_number=StringType(),
            firmware_revision=StringType()
        )),
        readonly=True
    )

    def read_full_identification(self):
        #url = self.io._make_url(f'/devices/identification')
        response = self.io.get('/devices/identification')
        self.log.info(f'number of ports is {len(response)}')
        ident_array = []
        for item in response:
            try:
                #device_alias = item["deviceAlias"]


                ident_array.append({
                    "vendor_id": int(item["vendorId"]),
                    "device_id": int(item["deviceId"]),
                    "port_number": int(item["portNumber"]),
                    "device_alias": str(item["deviceAlias"]),
                    "vendor_name": str(item["vendorName"]),
                    "vendor_text": str(item["vendorText"]),
                    "product_name": str(item["productName"]),
                    "product_id": str(item["productId"]),
                    "product_text": str(item["productText"]),
                    "serial_number": str(item["serialNumber"]),
                    "firmware_revision": str(item["firmwareRevision"]),
                })
            except(KeyError, TypeError, ValueError) as exc:
                raise CommunicationFailedError(
                    f"Invalid IO-Link identifaction response: {response!r}"
                )

        #self.log.info(response)
        return ident_array

