#!/usr/bin/env python
# *****************************************************************************
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Module authors:
#   Oliver Paull <oliverp@ansto.gov.au>
# *****************************************************************************

from frappy.core import Communicator, Property, Parameter, StringType, FloatRange, IntRange, BoolType
from frappy.lib import parse_host_port
from frappy.errors import CommunicationFailedError, ImpossibleError
import requests, threading
from typing import Any
from urllib.parse import urljoin

class RestIO(Communicator):
    uri = Property("""uri for serial connection

                   one of the following:

                   - ``tcp://<host address>:<portnumber>`` (see :class:`frappy.lib.asynconn.AsynTcp`)

                   - ``serial://<serial device>?baudrate=<value>...`` (see :class:`frappy.lib.asynconn.AsynSerial`)
                   """, datatype=StringType())

    #ip_address = Property('IP address for connection', datatype=StringType())

    #port = Property('Port of the TCP connection', datatype=IntRange())

    base_url = Parameter('Base URL of the REST API',
                         datatype=StringType(),
                         readonly=False)

    is_connected = Parameter('connection state', datatype=BoolType(), readonly=False, default=False,
                             update_unchanged='never')

    timeout = Parameter(
        "HTTP request timeout",
        datatype=FloatRange(
            0.1,
            300,
            unit="s",
        ),
        readonly=False,
        default=5.0,
    )

    verify_tls = Parameter(
        "Verify HTTPS server certificates",
        datatype=BoolType(),
        readonly=False,
        default=True,
    )

    username = Parameter(
        "HTTP basic-authentication username",
        datatype=StringType(),
        readonly=False,
        default="",
    )

    password = Parameter(
        "HTTP basic-authentication password",
        datatype=StringType(),
        readonly=False,
        default="",
    )

    debug_http = Parameter(
        "HTTP debugging",
        datatype=BoolType(),
        default=False
    )

    def initModule(self):
        super().initModule()

        if self.uri.startswith('tcp://'):
            uri = self.uri[6:]
            self.ip_address, self.port = parse_host_port(uri)
            #self.ip_address = str(ip_address)
            #self.log.info(f'configured IP = {self.ip_address}')
            #self.port = int(port)
        else:
            raise ImpossibleError('uri needs to be of type tcp:// for Rest API.')

        self._lock = threading.RLock()

        self._session = requests.Session()
        self._connected = False
        self.base_url = f"http://{self.ip_address}/api/balluff/v1"

        self._session.headers.update({
            "Accept": "application/json",
            "User-Agent": "Frappy-RestIO/1.0",
        })
        self._session.trust_env = False
        
        if self.username:
            self._session.auth = (
                self.username,
                self.password,
            )

        # Ensure joining paths behaves consistently.
        self._base_url = self.base_url.rstrip("/") + "/"

        self.log.info(
            "REST communicator configured for %s",
            self._base_url,
        )

    def shutdownModule(self):
        session = getattr(self, "_session", None)

        if session is not None:
            session.close()

        self._connected = False

        try:
            super().shutdownModule()
        except AttributeError:
            # Some Frappy versions may not implement this hook.
            pass

    def read_is_connected(self):
        """Return whether the most recent REST request succeeded."""

        return self._connected

    def _make_url(self, path: str) -> str:
        ''' Combine base URL and API path'''
        return urljoin(
            self._base_url,
            path.lstrip("/")
        )

    def request(self, method: str, path: str, *,
                params: dict[str, Any] | None = None,
                data: dict[str, Any] | None = None,
                json_body: Any | None = None,
                headers: dict[str, Any] | None = None,
                expected_status: int | tuple[int, ...] | None = None):
        ''' Send a valid HTTP request and get a Response object'''

        url = self._make_url(path)

        with self._lock:
            try:
                response = self._session.request(
                    method=method, 
                    url=url,
                    params=params,
                    json=json_body,
                    headers=headers,
                    timeout=float(self.timeout),
                    verify=bool(self.verify_tls)
                )

            except requests.Timeout as exc:
                self._connected = False

                raise CommunicationFailedError(
                    f'Request timed out. {method} {url} in {self.timeout} s.'
                ) from exc

            except requests.ConnectionError as exc:
                self._connected = False

                raise CommunicationFailedError(
                    'No connection to REST API'
                ) from exc

            except requests.RequestException as exc:
                self._connected = False

                raise CommunicationFailedError(
                    f'REST request failed. {method} {url}: {exc}'
                ) from exc

        self._connected = True

        

        if expected_status is not None:
            if isinstance(expected_status, int):
                expected_status = (expected_status,)
                
            if response.status_code not in tuple(expected_status):
                raise CommunicationFailedError(
                    self._format_http_error(
                        method,
                        url,
                        response,
                    )
                )
        else:
            try:
                response.raise_for_status()
            except requests.HTTPError as exc:
                raise CommunicationFailedError(
                    self._format_http_error(
                        method,
                        url,
                        response,
                    )
                ) from exc

        # A successful DELETE, PUT, or POST may legitimately return no body.
        if response.status_code == 204 or not response.content:
            return None

        content_type = response.headers.get(
            "Content-Type",
            "",
        ).lower()

        if "json" in content_type:
            try:
                result = response.json()
            except requests.JSONDecodeError as exc:
                raise CommunicationFailedError(
                    f"REST API returned invalid JSON for "
                    f"{method} {url}: {response.text[:500]!r}"
                ) from exc
        else:
            # Some poorly configured APIs return JSON with text/plain.
            try:
                result = response.json()
            except requests.JSONDecodeError:
                result = response.text

        if self.debug_http:
            self.log.debug(
                "HTTP %s %s returned %d: %r",
                method,
                url,
                response.status_code,
                result,
            )

        return result

    @staticmethod
    def _format_http_error(
        method: str,
        url: str,
        response: requests.Response,
    ) -> str:
        body = response.text.strip()

        if len(body) > 500:
            body = body[:500] + "..."

        return (
            f"REST API returned HTTP "
            f"{response.status_code} for "
            f"{method} {url}: {body!r}"
        )

    def get(
        self,
        path: str,
        *,
        params: dict[str, Any] | None = None,
    ) -> Any:
        return self.request(
            "GET",
            path,
            params=params,
            expected_status=200,
        )

    def post(
        self,
        path: str,
        *,
        json_body: Any = None,
    ) -> Any:
        return self.request(
            "POST",
            path,
            json_body=json_body,
            expected_status=(200, 201, 202, 204),
        )

    def put(
        self,
        path: str,
        *,
        json_body: Any = None,
    ) -> Any:
        return self.request(
            "PUT",
            path,
            json_body=json_body,
            expected_status=(200, 201, 202, 204),
        )

    def patch(
        self,
        path: str,
        *,
        json_body: Any = None,
    ) -> Any:
        return self.request(
            "PATCH",
            path,
            json_body=json_body,
            expected_status=(200, 202, 204),
        )

    def delete(self, path: str) -> Any:
        return self.request(
            "DELETE",
            path,
            expected_status=(200, 202, 204),
        )

    def communicate(self, request: dict[str, Any]) -> Any:
        """Generic Communicator-compatible dispatcher.

        Prefer get(), post(), put(), patch(), and delete() in device drivers.
        """

        if not isinstance(request, dict):
            raise TypeError(
                "RestIO.communicate() expects a dictionary"
            )

        request = dict(request)

        try:
            method = request.pop("method")
            path = request.pop("path")
        except KeyError as exc:
            raise ValueError(
                "REST request requires 'method' and 'path'"
            ) from exc

        return self.request(
            method,
            path,
            **request,
        )


